// The Sendspin SDK headers come first: Arduino.h (DISPLAY) and ESPuino's values.h (STOP, PLAY) define
// macros that collide with enumerators declared in them. Don't let clang-format reorder this block.
// clang-format off
#include <sendspin/client.h>
#include <sendspin/config.h>
#include <sendspin/controller_role.h>
#include <sendspin/metadata_role.h>
#include <sendspin/player_role.h>

#include <Arduino.h>
#include "settings.h"

#include "Sendspin.h"
// clang-format on

#ifdef SENDSPIN_ENABLE

	#include "AudioPlayer.h"
	#include "Log.h"
	#include "System.h"
	#include "Web.h"
	#include "Wlan.h"
	#include "gitrevision.h"
	#include "values.h"

	#include <ESPmDNS.h>
	#include <Preferences.h>
	#include <WiFi.h>
	#include <algorithm>
	#include <atomic>
	#include <cmath>
	#include <driver/i2s_std.h>
	#include <esp_rom_gpio.h>
	#include <freertos/ringbuf.h>
	#include <soc/i2s_periph.h>

// values.h's STOP/PLAY would expand inside SendspinControllerCommand::STOP/PLAY below
constexpr uint8_t kTrackCommandStop = STOP;
	#undef STOP
	#undef PLAY

using namespace sendspin;

float Audio_GetVolume(float t); // AudioPlayer.cpp: ESPuino's volume curve, dB for 0..1
const String getHostname(); // Wlan.cpp

namespace {

constexpr const char *kWsPath = "/sendspin"; // fixed by sendspin-cpp's WebSocket server

// ---------------------------------------------------------------------------------------------------
// Output
//
// The audio library keeps I2S0 for the whole lifetime of its Audio object, so the stream is played on a
// second I2S port on the same pins: while streaming the GPIO matrix routes the pins to that port, and
// afterwards back to I2S0 (Bluetooth sink mode drives the pins from a second port the same way).
//
// Sendspin's sync needs to know when frames actually left the DAC. The output task writes exactly one
// full DMA buffer per i2s_channel_write() (padding with silence) and queues how many stream frames it
// holds; the I2S on_sent interrupt queues a timestamp per finished DMA buffer. As long as the task keeps
// the DMA fed, both queues advance in lockstep and each pair reads "N frames finished playing at T" (same
// scheme as ESPHome's i2s_audio speaker). If the DMA ever runs dry it replays a buffer, which shows as more
// sent events than written buffers; the pipeline is then restarted to realign the queues.
// ---------------------------------------------------------------------------------------------------
constexpr uint32_t kDmaDescNum = 6;
constexpr uint32_t kDmaFrameNum = 240; // 5 ms at 48 kHz per DMA buffer
constexpr size_t kInBytesPerFrame = 4; // stream: 16 bit stereo (the only format advertised)
constexpr size_t kOutBytesPerFrame = 8; // I2S: 32 bit stereo slots, same as the audio library
constexpr size_t kOutBufBytes = kDmaFrameNum * kOutBytesPerFrame;
constexpr size_t kRingBytes = 4096 * kInBytesPerFrame; // ~85 ms at 48 kHz between sync task and output
constexpr size_t kMaxWriteChunk = 2048;
constexpr UBaseType_t kQueueLen = kDmaDescNum * 2;
constexpr uint32_t kOutputTaskStack = 3072;
constexpr UBaseType_t kOutputTaskPriority = PlayerRoleConfig::DEFAULT_SYNC_TASK_PRIORITY + 1;

i2s_chan_handle_t txHandle = nullptr;
RingbufHandle_t ring = nullptr;
QueueHandle_t sentEvents = nullptr; // int64_t: esp_timer time a DMA buffer finished (from the ISR)
QueueHandle_t writeRecords = nullptr; // uint32_t: stream frames in each written DMA buffer
TaskHandle_t outputTask = nullptr;
std::atomic<uint32_t> sentCount {0};
std::atomic<bool> outputWanted {false}; // the output task runs the I2S port while set
std::atomic<bool> outputAccepting {false}; // on_audio_write() feeds the ring while set
std::atomic<bool> outputRunning {false}; // the output task currently owns the pins
std::atomic<uint32_t> outputSampleRate {48000};
std::atomic<bool> outputMuted {false};
int32_t outBuf[kDmaFrameNum * 2];

SendspinClient *client = nullptr;
PlayerRole *playerRole = nullptr;

bool IRAM_ATTR Sendspin_OnSent(i2s_chan_handle_t, i2s_event_data_t *, void *) {
	int64_t now = esp_timer_get_time();
	BaseType_t woken = pdFALSE;
	sentCount.fetch_add(1, std::memory_order_relaxed);
	if (xQueueIsQueueFullFromISR(sentEvents)) {
		int64_t dropped;
		xQueueReceiveFromISR(sentEvents, &dropped, &woken);
	}
	xQueueSendToBackFromISR(sentEvents, &now, &woken);
	return woken == pdTRUE;
}

// Same math as the audio library (Audio::calculateVolumeLimits()) so a volume step sounds the same for
// local playback and Sendspin. Q16: 65536 is unity gain.
int32_t Sendspin_GainQ16(uint8_t volume) {
	if (volume == 0) {
		return 0;
	}
	const float t = std::min(1.0f, static_cast<float>(volume) / AUDIOPLAYER_VOLUME_MAX);
	const float dB = std::fmin(std::fmax(Audio_GetVolume(t), -60.0f), 0.0f);
	return static_cast<int32_t>(lroundf(powf(10.0f, dB / 20.0f) * 65536.0f));
}

// Fills outBuf with one DMA buffer: as many stream frames as the ring holds, silence for the rest.
uint32_t Sendspin_FillBuffer(int32_t gainQ16, bool mono) {
	uint32_t frames = 0;
	while (frames < kDmaFrameNum) {
		size_t got = 0;
		auto *in = static_cast<int16_t *>(xRingbufferReceiveUpTo(ring, &got, 0, (kDmaFrameNum - frames) * kInBytesPerFrame));
		if (in == nullptr) {
			break;
		}
		const uint32_t n = got / kInBytesPerFrame;
		int32_t *out = &outBuf[frames * 2];
		for (uint32_t i = 0; i < n; i++) {
			int32_t left = in[2 * i];
			int32_t right = in[2 * i + 1];
			if (mono) {
				left = right = (left + right) / 2;
			}
			out[2 * i] = left * gainQ16;
			out[2 * i + 1] = right * gainQ16;
		}
		frames += n;
		vRingbufferReturnItem(ring, in);
	}
	memset(&outBuf[frames * 2], 0, (kDmaFrameNum - frames) * kOutBytesPerFrame);
	return frames;
}

void Sendspin_DrainRing(void) {
	size_t got = 0;
	void *item;
	while ((item = xRingbufferReceiveUpTo(ring, &got, 0, kRingBytes)) != nullptr) {
		vRingbufferReturnItem(ring, item);
	}
}

// Frames still queued in DMA when the port stops are reported as played now, so the sync task's
// "frames written but not yet played" accounting doesn't keep them forever.
void Sendspin_CreditPending(void) {
	uint32_t frames = 0;
	uint32_t record;
	while (xQueueReceive(writeRecords, &record, 0) == pdTRUE) {
		frames += record;
	}
	xQueueReset(sentEvents);
	if (frames > 0 && playerRole != nullptr) {
		playerRole->notify_audio_played(frames, esp_timer_get_time());
	}
}

// Preloads every DMA buffer with silence and starts the port. Returns the number of buffers written.
uint32_t Sendspin_StartDma(void) {
	xQueueReset(sentEvents);
	xQueueReset(writeRecords);
	sentCount = 0;
	memset(outBuf, 0, sizeof(outBuf));
	uint32_t written = 0;
	const uint32_t none = 0;
	for (uint32_t i = 0; i < kDmaDescNum; i++) {
		size_t loaded = 0;
		if (i2s_channel_preload_data(txHandle, outBuf, kOutBufBytes, &loaded) != ESP_OK || loaded != kOutBufBytes) {
			break;
		}
		xQueueSend(writeRecords, &none, 0);
		written++;
	}
	if (i2s_channel_enable(txHandle) != ESP_OK) {
		Log_Println("Sendspin: unable to start I2S", LOGLEVEL_ERROR);
	}
	return written;
}

bool Sendspin_OpenChannel(uint32_t sampleRate) {
	i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	chanCfg.dma_desc_num = kDmaDescNum;
	chanCfg.dma_frame_num = kDmaFrameNum;
	chanCfg.auto_clear = true;
	if (i2s_new_channel(&chanCfg, &txHandle, nullptr) != ESP_OK) {
		Log_Println("Sendspin: no free I2S port", LOGLEVEL_ERROR);
		txHandle = nullptr;
		return false;
	}

	i2s_std_config_t stdCfg = {};
	stdCfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate);
	#ifdef I2S_COMM_FMT_LSB_ENABLE
	stdCfg.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
	#else
	stdCfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
	#endif
	stdCfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
	stdCfg.gpio_cfg.bclk = static_cast<gpio_num_t>(I2S_BCLK);
	stdCfg.gpio_cfg.ws = static_cast<gpio_num_t>(I2S_LRC);
	stdCfg.gpio_cfg.dout = static_cast<gpio_num_t>(I2S_DOUT);
	stdCfg.gpio_cfg.din = I2S_GPIO_UNUSED;

	i2s_event_callbacks_t cbs = {};
	cbs.on_sent = Sendspin_OnSent;
	if (i2s_channel_init_std_mode(txHandle, &stdCfg) != ESP_OK || i2s_channel_register_event_callback(txHandle, &cbs, nullptr) != ESP_OK) {
		Log_Println("Sendspin: unable to configure I2S", LOGLEVEL_ERROR);
		i2s_del_channel(txHandle);
		txHandle = nullptr;
		return false;
	}
	return true;
}

void Sendspin_CloseChannel(void) {
	i2s_del_channel(txHandle);
	txHandle = nullptr;
	// i2s_del_channel() leaves the pins routed to this port; hand them back to the audio library's I2S0.
	esp_rom_gpio_connect_out_signal(I2S_BCLK, i2s_periph_signal[I2S_NUM_0].m_tx_bck_sig, false, false);
	esp_rom_gpio_connect_out_signal(I2S_LRC, i2s_periph_signal[I2S_NUM_0].m_tx_ws_sig, false, false);
	esp_rom_gpio_connect_out_signal(I2S_DOUT, i2s_periph_signal[I2S_NUM_0].data_out_sig, false, false);
}

// Plays the ring until the stream ends or its sample rate changes.
void Sendspin_RunOutput(uint32_t sampleRate) {
	uint32_t written = Sendspin_StartDma();
	uint8_t cachedVolume = 0xFF;
	int32_t gainQ16 = 0;

	while (outputWanted && outputSampleRate == sampleRate) {
		const uint8_t volume = AudioPlayer_GetCurrentVolume();
		if (volume != cachedVolume) {
			gainQ16 = Sendspin_GainQ16(volume);
			cachedVolume = volume;
		}
		const uint32_t frames = Sendspin_FillBuffer(outputMuted ? 0 : gainQ16, gPlayProperties.currentPlayMono);

		size_t bytesWritten = 0;
		const esp_err_t err = i2s_channel_write(txHandle, outBuf, kOutBufBytes, &bytesWritten, 100);
		xQueueSend(writeRecords, &frames, 0);
		written++;

		if (err != ESP_OK || bytesWritten != kOutBufBytes || sentCount.load(std::memory_order_relaxed) > written) {
			// DMA ran dry (or a write failed): realign both queues
			Log_Println("Sendspin: output underrun, resyncing", LOGLEVEL_DEBUG);
			i2s_channel_disable(txHandle);
			Sendspin_CreditPending();
			written = Sendspin_StartDma();
			continue;
		}

		int64_t finishedAt;
		uint32_t record;
		while (uxQueueMessagesWaiting(writeRecords) > 0 && xQueueReceive(sentEvents, &finishedAt, 0) == pdTRUE) {
			xQueueReceive(writeRecords, &record, 0);
			if (record > 0) {
				playerRole->notify_audio_played(record, finishedAt);
			}
		}
	}

	i2s_channel_disable(txHandle);
	Sendspin_CreditPending();
}

void Sendspin_OutputTask(void *) {
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		while (outputWanted) {
			const uint32_t sampleRate = outputSampleRate;
			if (!Sendspin_OpenChannel(sampleRate)) {
				outputWanted = false;
				break;
			}
			outputRunning = true;
			Log_Printf(LOGLEVEL_DEBUG, "Sendspin: output started at %u Hz", sampleRate);
			Sendspin_RunOutput(sampleRate);
			Sendspin_CloseChannel();
			outputRunning = false;
			Log_Println("Sendspin: output stopped", LOGLEVEL_DEBUG);
		}
		Sendspin_DrainRing();
	}
}

void Sendspin_StartOutput(uint32_t sampleRate) {
	outputSampleRate = sampleRate;
	outputAccepting = true;
	outputWanted = true;
	xTaskNotifyGive(outputTask);
}

void Sendspin_StopOutput(void) {
	outputAccepting = false;
	outputWanted = false;
}

// ---------------------------------------------------------------------------------------------------
// Session state (main loop only, unless atomic)
// ---------------------------------------------------------------------------------------------------
std::atomic<bool> isSource {false}; // the Sendspin stream is the current audio source
bool lastLocalActive = false;
QueueHandle_t trackCommands = nullptr; // uint8_t track commands forwarded from any task
std::atomic<int16_t> pendingLocalVolume {-1};
uint8_t serverVolume = 0; // last volume reported to / received from the server (0..100)
Preferences prefsSendspin;

// Web UI status: written by the main loop, read by the web server task through Sendspin_GetStatus()
SemaphoreHandle_t statusMutex = nullptr;
SendspinStatus status;
bool trustPaired = false; // last on_trust_changed(); describes the connection only while connected
bool metadataChanged = false;
uint32_t lastStatusRefresh = 0;
constexpr uint32_t kStatusRefreshMs = 500;
std::atomic<bool> unpairedAccess {false};
std::atomic<int8_t> pendingUnpairedAccess {-1}; // set from the web task, applied in Sendspin_Cyclic()
// what the web UI's player view last showed, to push a trackinfo update when it changes
bool shownIsSource = false;
bool shownPlaying = false;

class StatusLock {
public:
	StatusLock() {
		xSemaphoreTake(statusMutex, portMAX_DELAY);
	}
	~StatusLock() {
		xSemaphoreGive(statusMutex);
	}
};

uint8_t Sendspin_ToEspuinoVolume(uint8_t volume100) {
	return (volume100 * AUDIOPLAYER_VOLUME_MAX + 50) / 100;
}

uint8_t Sendspin_ToSendspinVolume(uint8_t volume) {
	return std::min<uint32_t>(100, (volume * 100u + AUDIOPLAYER_VOLUME_MAX / 2) / AUDIOPLAYER_VOLUME_MAX);
}

bool Sendspin_LocalPlaybackActive(void) {
	return gPlayProperties.playMode != NO_PLAYLIST && gPlayProperties.playMode != PLAYLIST_BUSY && !gPlayProperties.pausePlay;
}

const char *Sendspin_CodecName(std::optional<SendspinCodecFormat> codec) {
	if (!codec) {
		return "?";
	}
	switch (*codec) {
		case SendspinCodecFormat::FLAC:
			return "flac";
		case SendspinCodecFormat::OPUS:
			return "opus";
		case SendspinCodecFormat::PCM:
			return "pcm";
		default:
			return "unsupported";
	}
}

void Sendspin_OnStreamStart(void) {
	const ServerPlayerStreamObject &params = playerRole->get_current_stream_params();
	const uint32_t sampleRate = params.sample_rate.value_or(48000);
	Log_Printf(LOGLEVEL_NOTICE, "Sendspin: stream started (%s, %u Hz, %u ch, %u bit)", Sendspin_CodecName(params.codec), sampleRate, params.channels.value_or(0), params.bit_depth.value_or(0));
	if (params.channels.value_or(2) != 2 || params.bit_depth.value_or(16) != 16) {
		Log_Println("Sendspin: unexpected stream format, expected 16 bit stereo", LOGLEVEL_ERROR);
	}

	if (!isSource && Sendspin_LocalPlaybackActive()) {
		AudioPlayer_SetTrackControl(PAUSEPLAY); // before isSource is set, so it isn't forwarded to the server
	}
	isSource = true;
	// The pause above is processed asynchronously; only a later inactive -> active edge hands back.
	lastLocalActive = Sendspin_LocalPlaybackActive();
	Sendspin_StartOutput(sampleRate);
}

void Sendspin_OnStreamEnd(void) {
	Log_Println("Sendspin: stream ended", LOGLEVEL_NOTICE);
	Sendspin_StopOutput();
	// isSource stays set: a paused Sendspin stream still owns the play button
}

// Local playback was started (RFID, web UI, ...) while Sendspin is the source: hand the speaker back.
void Sendspin_YieldToLocalPlayback(void) {
	Log_Println("Sendspin: local playback started, leaving the Sendspin group", LOGLEVEL_NOTICE);
	isSource = false;
	Sendspin_StopOutput();
	if (client->is_connected()) {
		client->leave();
	}
}

void Sendspin_ForwardTrackCommand(uint8_t trackCommand) {
	ControllerRole *controller = client->controller();
	if (controller == nullptr || !client->is_connected()) {
		return;
	}
	const auto &supported = controller->get_controller_state().supported_commands;
	auto isSupported = [&supported](SendspinControllerCommand cmd) {
		return std::find(supported.begin(), supported.end(), cmd) != supported.end();
	};

	SendspinControllerCommand cmd;
	switch (trackCommand) {
		case PAUSEPLAY: {
			const auto state = client->get_group_state().playback_state;
			const bool playing = state ? (*state == SendspinPlaybackState::PLAYING) : outputWanted.load();
			cmd = playing ? SendspinControllerCommand::PAUSE : SendspinControllerCommand::PLAY;
			if (playing && !isSupported(cmd)) {
				cmd = SendspinControllerCommand::STOP;
			}
			break;
		}
		case NEXTTRACK:
			cmd = SendspinControllerCommand::NEXT;
			break;
		case PREVIOUSTRACK:
			cmd = SendspinControllerCommand::PREVIOUS;
			break;
		case kTrackCommandStop:
			cmd = SendspinControllerCommand::STOP;
			break;
		default:
			return;
	}
	if (!isSupported(cmd)) {
		Log_Printf(LOGLEVEL_INFO, "Sendspin: server doesn't support command %u", static_cast<uint8_t>(cmd));
		return;
	}
	ClientCommandControllerObject command;
	command.command = cmd;
	if (!controller->send_command(command)) {
		Log_Println("Sendspin: command queue full", LOGLEVEL_NOTICE);
	}
}

// ---------------------------------------------------------------------------------------------------
// SDK glue
// ---------------------------------------------------------------------------------------------------
class PlayerListener : public PlayerRoleListener {
public:
	// Sync task
	size_t on_audio_write(uint8_t *data, size_t length, uint32_t timeout_ms) override {
		if (!outputAccepting) {
			// Speaker handed back to local playback while the server still sends: drop at real-time pace
			const uint32_t ms = (length / kInBytesPerFrame) * 1000u / std::max<uint32_t>(1, outputSampleRate);
			vTaskDelay(pdMS_TO_TICKS(std::min(ms, timeout_ms)) + 1);
			return length;
		}
		const size_t chunk = std::min(length, kMaxWriteChunk) & ~(kInBytesPerFrame - 1);
		if (chunk == 0) {
			return length; // less than one frame; can't happen with whole-frame writes
		}
		if (xRingbufferSend(ring, data, chunk, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
			return 0;
		}
		return chunk;
	}

	// Main loop (client->loop())
	void on_stream_start() override {
		Sendspin_OnStreamStart();
	}
	void on_stream_end() override {
		Sendspin_OnStreamEnd();
	}
	void on_volume_changed(uint8_t volume) override {
		serverVolume = volume;
		const uint8_t local = Sendspin_ToEspuinoVolume(volume);
		if (local != AudioPlayer_GetCurrentVolume()) {
			AudioPlayer_SetVolume(local); // reports back via Sendspin_OnLocalVolumeChanged() if clamped
		}
	}
	void on_mute_changed(bool muted) override {
		outputMuted = muted;
	}
};

class MetadataListener : public MetadataRoleListener {
public:
	// Each call carries the full track state, so every field is overwritten
	void on_metadata(const ServerMetadataStateObject &metadata) override {
		if (metadata.title) {
			Log_Printf(LOGLEVEL_NOTICE, "Sendspin: now playing %s - %s", metadata.artist.value_or("?").c_str(), metadata.title->c_str());
		}
		StatusLock lock;
		status.title = metadata.title.value_or("").c_str();
		status.artist = metadata.artist.value_or("").c_str();
		status.album = metadata.album.value_or("").c_str();
		status.artworkUrl = metadata.artwork_url.value_or("").c_str();
		metadataChanged = true;
	}
	void on_metadata_clear() override {
		StatusLock lock;
		status.title = "";
		status.artist = "";
		status.album = "";
		status.artworkUrl = "";
		metadataChanged = true;
	}
};

class ClientListener : public SendspinClientListener {
public:
	void on_group_update(const GroupUpdateObject &group) override {
		if (group.playback_state) {
			Log_Printf(LOGLEVEL_INFO, "Sendspin: group %s", *group.playback_state == SendspinPlaybackState::PLAYING ? "playing" : "stopped");
		}
	}
	void on_request_high_performance() override {
		Wlan_SetPowerSave(false);
	}
	void on_release_high_performance() override {
		Wlan_SetPowerSave(true);
	}
	void on_trust_changed(ConnectionTrust trust) override {
		Log_Printf(LOGLEVEL_NOTICE, "Sendspin: server connected (%s)", trust == ConnectionTrust::USER ? "paired" : "unpaired");
		trustPaired = (trust == ConnectionTrust::USER);
		lastStatusRefresh = 0; // refresh the web UI status right away
	}
	void on_pairing_succeeded(const std::string &serverId) override {
		Log_Printf(LOGLEVEL_NOTICE, "Sendspin: paired with server %s", serverId.c_str());
	}
	void on_pairing_failed(const std::string &serverId, SendspinPairAbortReason reason) override {
		Log_Printf(LOGLEVEL_ERROR, "Sendspin: pairing with server %s failed (%u)", serverId.c_str(), static_cast<uint8_t>(reason));
	}
};

class NetworkProvider : public SendspinNetworkProvider {
public:
	bool is_network_ready() override {
		return Wlan_IsConnected();
	}
};

// Keypair, pairing records and the pairing PSK live in their own NVS namespace.
class NvsPersistence : public SendspinPersistenceProvider {
public:
	std::optional<std::vector<uint8_t>> load_blob(const std::string &key) override {
		if (!prefsSendspin.isKey(key.c_str())) {
			return std::nullopt;
		}
		std::vector<uint8_t> blob(prefsSendspin.getBytesLength(key.c_str()));
		prefsSendspin.getBytes(key.c_str(), blob.data(), blob.size());
		return blob;
	}
	bool save_blob(const std::string &key, const uint8_t *data, size_t len) override {
		if (len == 0) {
			return prefsSendspin.remove(key.c_str()) || !prefsSendspin.isKey(key.c_str());
		}
		return prefsSendspin.putBytes(key.c_str(), data, len) == len;
	}
};

// Updates the web UI snapshot and pushes a trackinfo update when what the player view shows changed.
void Sendspin_RefreshStatus(void) {
	const bool connected = client->is_connected();
	const auto playbackState = client->get_group_state().playback_state;
	const bool playing = connected && playbackState && *playbackState == SendspinPlaybackState::PLAYING;
	String serverName;
	if (connected) {
		if (const auto info = client->get_server_information()) {
			serverName = info->name.c_str();
		}
	}
	const MetadataRole *metadata = client->metadata();

	bool trackChanged;
	{
		StatusLock lock;
		status.connected = connected;
		status.paired = connected && trustPaired;
		status.playing = playing;
		status.isSource = isSource;
		status.unpairedAccess = unpairedAccess;
		status.serverName = serverName;
		status.progressMs = metadata ? metadata->get_track_progress_ms() : 0;
		status.durationMs = metadata ? metadata->get_track_duration_ms() : 0;
		trackChanged = metadataChanged;
		metadataChanged = false;
	}

	const bool sourceChanged = (isSource != shownIsSource);
	if (sourceChanged || (isSource && (trackChanged || playing != shownPlaying))) {
		Web_SendWebsocketData(0, WebsocketCodeType::TrackInfo);
		if (sourceChanged && !isSource) {
			Web_SendWebsocketData(0, WebsocketCodeType::CoverImg); // back to the local cover
		}
		shownIsSource = isSource;
		shownPlaying = playing;
	}
}

PlayerListener playerListener;
MetadataListener metadataListener;
ClientListener clientListener;
NetworkProvider networkProvider;
NvsPersistence persistence;

} // namespace

void Sendspin_Init(void) {
	if (System_GetOperationMode() != OPMODE_NORMAL) {
		return; // Bluetooth modes switch WiFi off
	}

	ring = xRingbufferCreateWithCaps(kRingBytes, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	sentEvents = xQueueCreate(kQueueLen, sizeof(int64_t));
	writeRecords = xQueueCreate(kQueueLen, sizeof(uint32_t));
	trackCommands = xQueueCreate(4, sizeof(uint8_t));
	statusMutex = xSemaphoreCreateMutex();
	if (ring == nullptr || sentEvents == nullptr || writeRecords == nullptr || trackCommands == nullptr || statusMutex == nullptr
		|| xTaskCreatePinnedToCore(Sendspin_OutputTask, "sendspinOut", kOutputTaskStack, nullptr, kOutputTaskPriority, &outputTask, 1) != pdPASS) {
		Log_Println("Sendspin: out of memory", LOGLEVEL_ERROR);
		return;
	}

	prefsSendspin.begin("sendspin", false);

	SendspinClientConfig config;
	const String hostname = getHostname();
	config.name = hostname.length() > 0 ? hostname.c_str() : "ESPuino";
	config.product_name = "ESPuino";
	config.manufacturer = "ESPuino";
	config.software_version = softwareRevisionShort;
	config.mac_address = WiFi.macAddress().c_str();
	// Task stacks stay in internal RAM: sdkconfig doesn't set CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY, and a
	// PSRAM stack then fails xTaskCreateStatic's stack check (crash loop as soon as WiFi is up and httpd starts).
	config.server_max_connections = 2; // lwIP sockets are shared with web server, FTP and MQTT
	client = new SendspinClient(std::move(config));

	PlayerRoleConfig playerConfig;
	playerConfig.audio_formats = {
		{SendspinCodecFormat::FLAC, 2, 48000, 16},
		{SendspinCodecFormat::FLAC, 2, 44100, 16},
		{ SendspinCodecFormat::PCM, 2, 48000, 16},
		{ SendspinCodecFormat::PCM, 2, 44100, 16},
	};
	playerConfig.audio_buffer_capacity = 512 * 1024; // in PSRAM
	PlayerRole &player = client->add_player(std::move(playerConfig));
	player.set_listener(&playerListener);
	playerRole = &player;

	client->add_metadata().set_listener(&metadataListener);
	client->add_controller();

	client->set_listener(&clientListener);
	client->set_network_provider(&networkProvider);
	client->set_persistence_provider(&persistence);
	unpairedAccess = prefsSendspin.getBool("unpaired", false);
	client->set_unpaired_access_enabled(unpairedAccess);

	serverVolume = Sendspin_ToSendspinVolume(AudioPlayer_GetCurrentVolume());
	player.update_volume(serverVolume);

	if (!client->start()) {
		Log_Println("Sendspin: unable to start client", LOGLEVEL_ERROR);
		delete client;
		client = nullptr;
		playerRole = nullptr;
		return;
	}
	Log_Printf(LOGLEVEL_NOTICE, "Sendspin: client started (id %s)", client->client_id().c_str());
	const auto token = client->pairing_token();
	if (token) {
		Log_Printf(LOGLEVEL_NOTICE, "Sendspin: pairing token %s", token->c_str());
	}
	StatusLock lock;
	status.running = true;
	status.clientId = client->client_id().c_str();
	status.pairingToken = token ? token->c_str() : "";
}

void Sendspin_AdvertiseMdns(void) {
	if (client == nullptr) {
		return;
	}
	MDNS.addService("sendspin", "tcp", SendspinClientConfig::DEFAULT_SERVER_PORT);
	MDNS.addServiceTxt("sendspin", "tcp", "path", kWsPath);
}

void Sendspin_Cyclic(void) {
	if (client == nullptr) {
		return;
	}
	client->loop();

	uint8_t trackCommand;
	while (xQueueReceive(trackCommands, &trackCommand, 0) == pdTRUE) {
		Sendspin_ForwardTrackCommand(trackCommand);
	}

	const int16_t localVolume = pendingLocalVolume.exchange(-1);
	if (localVolume >= 0) {
		// Keep the server's exact value when it maps to the same step, so its slider doesn't jump
		if (Sendspin_ToEspuinoVolume(serverVolume) != localVolume) {
			serverVolume = Sendspin_ToSendspinVolume(localVolume);
		}
		playerRole->update_volume(serverVolume);
	}

	const bool localActive = Sendspin_LocalPlaybackActive();
	if (isSource && localActive && !lastLocalActive) {
		Sendspin_YieldToLocalPlayback();
	}
	lastLocalActive = localActive;

	if (isSource && !outputWanted && !client->is_connected()) {
		isSource = false; // server gone while paused: buttons control local playback again
	}
	if (outputWanted) {
		System_UpdateActivityTimer(); // streaming counts as activity, no sleep timer
	}

	const int8_t unpaired = pendingUnpairedAccess.exchange(-1);
	if (unpaired >= 0) {
		prefsSendspin.putBool("unpaired", unpaired == 1);
		client->set_unpaired_access_enabled(unpaired == 1);
		Log_Printf(LOGLEVEL_NOTICE, "Sendspin: unpaired access %s", unpaired == 1 ? "allowed" : "blocked");
	}

	if (metadataChanged || millis() - lastStatusRefresh >= kStatusRefreshMs) {
		lastStatusRefresh = millis();
		Sendspin_RefreshStatus();
	}
}

void Sendspin_Exit(void) {
	if (client == nullptr) {
		return;
	}
	Sendspin_StopOutput();
	client->stop();
	for (uint8_t i = 0; i < 50 && outputRunning; i++) {
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	prefsSendspin.end();
}

bool Sendspin_IsSource(void) {
	return isSource;
}

bool Sendspin_HandleTrackCommand(uint8_t trackCommand) {
	if (!isSource || trackCommands == nullptr) {
		return false;
	}
	switch (trackCommand) {
		case PAUSEPLAY:
		case NEXTTRACK:
		case PREVIOUSTRACK:
		case kTrackCommandStop:
			xQueueSend(trackCommands, &trackCommand, 0);
			return true;
		default:
			return false;
	}
}

void Sendspin_OnLocalVolumeChanged(uint8_t volume) {
	pendingLocalVolume = volume;
}

void Sendspin_GetStatus(SendspinStatus &out) {
	if (statusMutex == nullptr) {
		out = SendspinStatus(); // not running (Bluetooth mode or init failed)
		return;
	}
	StatusLock lock;
	out = status;
}

void Sendspin_SetUnpairedAccess(bool enabled) {
	unpairedAccess = enabled;
	pendingUnpairedAccess = enabled ? 1 : 0;
}

bool Sendspin_GetUnpairedAccess(void) {
	return unpairedAccess;
}

#endif
