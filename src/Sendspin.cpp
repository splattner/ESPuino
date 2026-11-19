// The Sendspin SDK headers come first: Arduino.h (DISPLAY) and ESPuino's values.h (STOP, PLAY) define
// macros that collide with enumerators declared in them. Don't let clang-format reorder this block.
// clang-format off
#include <sendspin/client.h>
#include <sendspin/color_role.h>
#include <sendspin/config.h>
#include <sendspin/controller_role.h>
#include <sendspin/metadata_role.h>
#include <sendspin/player_role.h>
#include <sendspin/visualizer_role.h>

#include <Arduino.h>
#include "settings.h"

#include "Sendspin.h"
// clang-format on

#ifdef SENDSPIN_ENABLE

	#include "AudioPlayer.h"
	#include "Log.h"
	#include "Mqtt.h"
	#include "System.h"
	#include "Web.h"
	#include "Wlan.h"
	#include "gitrevision.h"
	#include "values.h"

	#include <ArduinoJson.h>
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
std::atomic<bool> fadeOutRequested {false}; // fade out, then stop the output (Sendspin_StopOutput(true))
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

// Fills outBuf with one DMA buffer of unscaled samples: as many stream frames as the ring holds, silence
// for the rest.
uint32_t Sendspin_FillBuffer(bool mono) {
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
			out[2 * i] = left;
			out[2 * i + 1] = right;
		}
		frames += n;
		vRingbufferReturnItem(ring, in);
	}
	memset(&outBuf[frames * 2], 0, (kDmaFrameNum - frames) * kOutBytesPerFrame);
	return frames;
}

// Scales the first `frames` frames of outBuf, moving the gain linearly from gainFrom to gainTo (Q16), so
// volume changes and fades never jump within a sample.
void Sendspin_ApplyGain(uint32_t frames, int32_t gainFrom, int32_t gainTo) {
	if (frames == 0) {
		return;
	}
	const int32_t step = (gainTo - gainFrom) / static_cast<int32_t>(frames);
	int32_t gain = gainFrom;
	for (uint32_t i = 0; i < frames; i++) {
		gain += step;
		const int32_t g = (i + 1 == frames) ? gainTo : gain;
		outBuf[2 * i] *= g;
		outBuf[2 * i + 1] *= g;
	}
}

// Fade envelope (Q16, per DMA buffer of ~5 ms): fade in over ~300 ms after a start or a dropout, fade out
// over ~50 ms on a stop (enough for the audio still in the ring)
constexpr int32_t kUnityQ16 = 65536;
constexpr int32_t kFadeInStep = kUnityQ16 / 60;
constexpr int32_t kFadeOutStep = kUnityQ16 / 10;

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
	int32_t volumeGain = 0;
	int32_t fade = 0; // envelope, Q16
	int32_t lastGain = 0; // effective gain at the end of the previous buffer

	while (outputWanted && outputSampleRate == sampleRate) {
		const uint8_t volume = AudioPlayer_GetCurrentVolume();
		if (volume != cachedVolume) {
			volumeGain = Sendspin_GainQ16(volume);
			cachedVolume = volume;
		}
		const uint32_t frames = Sendspin_FillBuffer(gPlayProperties.currentPlayMono);

		const bool fadingOut = fadeOutRequested.load();
		if (frames < kDmaFrameNum) {
			fade = 0; // ran dry: ramp down to the gap instead of cutting off, fade in again afterwards
		} else if (fadingOut) {
			fade = std::max<int32_t>(0, fade - kFadeOutStep);
		} else {
			fade = std::min<int32_t>(kUnityQ16, fade + kFadeInStep);
		}
		const int32_t gain = outputMuted ? 0 : static_cast<int32_t>((static_cast<int64_t>(volumeGain) * fade) >> 16);
		Sendspin_ApplyGain(frames, lastGain, gain);
		lastGain = gain;
		if (fadingOut && fade == 0 && fadeOutRequested.exchange(false)) {
			outputWanted = false; // faded out: this buffer (silent at its end) is the last one
		}

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
			fade = 0;
			lastGain = 0;
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

	// Let the DMA play out what's queued (e.g. the end of a fade-out) before stopping, so the output
	// doesn't cut off mid-signal
	memset(outBuf, 0, sizeof(outBuf));
	for (uint32_t i = 0; i < kDmaDescNum; i++) {
		size_t bytesWritten = 0;
		if (i2s_channel_write(txHandle, outBuf, kOutBufBytes, &bytesWritten, 100) != ESP_OK) {
			break;
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
	fadeOutRequested = false; // a stop that is still fading out turns into a fade in again
	outputSampleRate = sampleRate;
	outputAccepting = true;
	outputWanted = true;
	xTaskNotifyGive(outputTask);
}

// fade: play what's left in the ring with a ~50 ms fade-out (the output stops itself). Otherwise stop
// at once (after the current DMA buffer).
void Sendspin_StopOutput(bool fade) {
	outputAccepting = false;
	if (fade && outputWanted) {
		fadeOutRequested = true;
	} else {
		outputWanted = false;
	}
}

// Waits up to timeoutMs for the output task to release the I2S pins.
void Sendspin_WaitOutputStopped(uint32_t timeoutMs) {
	for (uint32_t waited = 0; waited < timeoutMs && (outputRunning || outputWanted); waited += 10) {
		vTaskDelay(pdMS_TO_TICKS(10));
	}
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
std::atomic<bool> enabledSetting {true};
constexpr const char *kPrefsNamespace = "sendspin";
constexpr const char *kPrefsEnabled = "enabled";
std::atomic<int8_t> pendingUnpairedAccess {-1}; // set from the web task, applied in Sendspin_Cyclic()
// Sleep modes (main loop)
uint32_t lastOwnCommandAt = 0; // millis() of the last play/pause/next/previous/stop sent to the server
constexpr uint32_t kOwnCommandMs = 3000; // track changes and stops this soon after are the user's doing
bool trackChangedForSleep = false; // set by on_metadata() when a different track started
std::string lastTrackKey; // artist + title of the current track
bool lastGroupPlaying = false;
bool eo5tArmed = false;
std::atomic<uint8_t> sleepTracksLeft {0};
bool sleepRequested = false;

// LED ring: written by the main loop and the SDK's threads, read by the LED task
std::atomic<bool> ledPlaying {false};
std::atomic<uint8_t> ledProgress {0xFF};
std::atomic<uint8_t> ledMode {static_cast<uint8_t>(SendspinLedMode::Progress)};
std::atomic<bool> visualizerStreaming {false};
std::atomic<uint32_t> spectrumAt {0}; // millis() of the last spectrum frame
std::atomic<uint32_t> beatAt {0}; // millis() of the last beat
std::atomic<uint8_t> spectrumLevels[kSendspinSpectrumBins] = {};
std::atomic<bool> haveColors {false};
std::atomic<uint32_t> primaryColor {0}; // 0xRRGGBB
std::atomic<uint32_t> accentColor {0};
constexpr uint32_t kSpectrumStaleMs = 500; // no frames for this long: fall back to the progress animation
constexpr uint32_t kBeatFlashMs = 150;
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
	Sendspin_StopOutput(true);
	// isSource stays set: a paused Sendspin stream still owns the play button
}

// Local playback was started (RFID, web UI, ...) while Sendspin is the source: hand the speaker back.
void Sendspin_YieldToLocalPlayback(void) {
	Log_Println("Sendspin: local playback started, leaving the Sendspin group", LOGLEVEL_NOTICE);
	isSource = false;
	Sendspin_StopOutput(true);
	if (client->is_connected()) {
		client->leave();
	}
}

void Sendspin_RequestSleep(const char *reason) {
	if (sleepRequested) {
		return;
	}
	sleepRequested = true;
	Log_Printf(LOGLEVEL_NOTICE, "Sendspin: sleep timer: %s, going to sleep", reason);
	System_RequestSleep(); // powering down fades out and disconnects from the server (Sendspin_Exit())
}

// The sleep modes, applied to the stream: called from the main loop with the track/group changes seen
void Sendspin_CheckSleepModes(bool groupStopped) {
	const bool eot = gPlayProperties.sleepAfterCurrentTrack;
	const bool eop = gPlayProperties.sleepAfterPlaylist;
	const bool eo5t = gPlayProperties.playUntilTrackNumber > 0;
	if (eo5t && !eo5tArmed) {
		sleepTracksLeft = 5; // counted from when the mode was set
	}
	eo5tArmed = eo5t;
	if (!eo5t) {
		sleepTracksLeft = 0;
	}

	const bool trackChanged = trackChangedForSleep;
	trackChangedForSleep = false;
	if (!isSource || (millis() - lastOwnCommandAt < kOwnCommandMs)) {
		return; // skipping a track or pausing on ESPuino doesn't count as a track/playlist ending
	}
	if (trackChanged) {
		if (eot) {
			Sendspin_RequestSleep("track ended");
		} else if (eo5t && sleepTracksLeft > 0 && --sleepTracksLeft == 0) {
			Sendspin_RequestSleep("5 tracks played");
		}
	}
	if (groupStopped && (eot || eop || eo5t)) {
		Sendspin_RequestSleep("playback ended");
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
	lastOwnCommandAt = millis();
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
		if (metadata.title) {
			const std::string key = metadata.artist.value_or("") + "\n" + *metadata.title;
			if (key != lastTrackKey) {
				trackChangedForSleep = !lastTrackKey.empty(); // the first track of a session isn't a change
				lastTrackKey = key;
			}
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

	#if defined(NEOPIXEL_ENABLE) && defined(SENDSPIN_ENABLE_VISUALIZER)
// Data callbacks run on the SDK's visualizer thread at the frame's display time
class VisualizerListener : public VisualizerRoleListener {
public:
	void on_spectrum(int64_t, const std::vector<uint16_t> &bins) override {
		const size_t n = std::min<size_t>(bins.size(), kSendspinSpectrumBins);
		for (size_t i = 0; i < n; i++) {
			// 0..65535 is -60..0 dB; below -50 dB is shown as off
			const uint32_t level = bins[i] >> 8;
			spectrumLevels[i] = level <= 42 ? 0 : (level - 42) * 255 / 213;
		}
		spectrumAt = millis();
	}
	void on_beat(int64_t, bool) override {
		beatAt = millis();
	}
	void on_visualizer_stream_start(const ServerVisualizerStreamObject &) override {
		visualizerStreaming = true;
	}
	void on_visualizer_stream_end() override {
		visualizerStreaming = false;
	}
	void on_visualizer_stream_clear() override {
		for (auto &level : spectrumLevels) {
			level = 0;
		}
	}
};
	#endif

	#if defined(NEOPIXEL_ENABLE) && defined(SENDSPIN_ENABLE_COLOR)
class ColorListener : public ColorRoleListener {
public:
	void on_color(const ServerColorStateObject &color) override {
		const auto &primary = color.primary ? color.primary : color.accent;
		const auto &accent = color.accent ? color.accent : color.primary;
		if (!primary) {
			haveColors = false;
			return;
		}
		primaryColor = Sendspin_PackRgb(*primary);
		accentColor = Sendspin_PackRgb(*accent);
		haveColors = true;
	}
	void on_color_clear() override {
		haveColors = false;
	}

private:
	static uint32_t Sendspin_PackRgb(const RgbColor &rgb) {
		return (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
	}
};
	#endif

class ClientListener : public SendspinClientListener {
public:
	// Servers repeat the group state with every group update (e.g. on each volume change); log changes only
	void on_group_update(const GroupUpdateObject &group) override {
		if (group.playback_state && group.playback_state != loggedState) {
			loggedState = group.playback_state;
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
		loggedState.reset(); // a new connection reports its state afresh
		trustPaired = (trust == ConnectionTrust::USER);
		lastStatusRefresh = 0; // refresh the web UI status right away
	}
	void on_pairing_succeeded(const std::string &serverId) override {
		Log_Printf(LOGLEVEL_NOTICE, "Sendspin: paired with server %s", serverId.c_str());
	}
	void on_pairing_failed(const std::string &serverId, SendspinPairAbortReason reason) override {
		Log_Printf(LOGLEVEL_ERROR, "Sendspin: pairing with server %s failed (%u)", serverId.c_str(), static_cast<uint8_t>(reason));
	}

private:
	std::optional<SendspinPlaybackState> loggedState;
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

	#ifdef MQTT_ENABLE
// MQTT state topics. Self-deduplicating: called on every status refresh, publishes what changed (and the
// track progress at most every kMqttProgressMs while playing).
std::atomic<bool> mqttForce {true};
bool mqttIsSource = false;
String mqttTrack;
String mqttPausePlay;
String mqttStateKey;
uint32_t mqttProgressAt = 0;
constexpr uint32_t kMqttProgressMs = 5000;

void Sendspin_PublishMqtt(const SendspinStatus &s) {
	const bool force = mqttForce.exchange(false);

	if (force || isSource != mqttIsSource) {
		if (publishMqtt(topicSource, isSource ? "sendspin" : "local", false)) {
			if (!isSource && mqttIsSource) {
				// back to local playback: its track and state replace the stream's
				publishMqtt(topicTrack, gPlayProperties.title, false);
				publishMqtt(topicPausePlay, gPlayProperties.playMode == NO_PLAYLIST ? "idle" : (gPlayProperties.pausePlay ? "pause" : "play"), false);
			}
			mqttIsSource = isSource;
			mqttTrack = "";
			mqttPausePlay = "";
		}
	}

	if (isSource) {
		String track = s.title.length() == 0 ? String("Sendspin") : (s.artist.length() > 0 ? s.artist + " - " + s.title : s.title);
		if ((force || track != mqttTrack) && publishMqtt(topicTrack, track.c_str(), false)) {
			mqttTrack = track;
		}
		const char *pausePlay = s.playing ? "play" : "pause";
		if ((force || mqttPausePlay != pausePlay) && publishMqtt(topicPausePlay, pausePlay, false)) {
			mqttPausePlay = pausePlay;
		}
	}

	JsonDocument doc;
	doc["connected"] = s.connected;
	doc["playing"] = s.playing;
	doc["server"] = s.serverName;
	doc["title"] = s.title;
	doc["artist"] = s.artist;
	doc["album"] = s.album;
	doc["artworkUrl"] = s.artworkUrl;
	doc["durationMs"] = s.durationMs;
	String key;
	serializeJson(doc, key); // everything but the progress
	const bool progressDue = s.playing && (millis() - mqttProgressAt >= kMqttProgressMs);
	if (!force && key == mqttStateKey && !progressDue) {
		return;
	}
	doc["progressMs"] = s.progressMs;
	String payload;
	serializeJson(doc, payload);
	if (publishMqtt(topicSendspinState, payload.c_str(), false)) {
		mqttStateKey = key;
		mqttProgressAt = millis();
	}
}
	#endif

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
	const uint32_t progressMs = metadata ? metadata->get_track_progress_ms() : 0;
	const uint32_t durationMs = metadata ? metadata->get_track_duration_ms() : 0;
	ledPlaying = playing;
	ledProgress = (durationMs > 0) ? std::min<uint64_t>(100, static_cast<uint64_t>(progressMs) * 100 / durationMs) : 0xFF;

	bool trackChanged;
	{
		StatusLock lock;
		status.connected = connected;
		status.paired = connected && trustPaired;
		status.playing = playing;
		status.isSource = isSource;
		status.unpairedAccess = unpairedAccess;
		status.serverName = serverName;
		status.progressMs = progressMs;
		status.durationMs = durationMs;
		trackChanged = metadataChanged;
		metadataChanged = false;
	}
	#ifdef MQTT_ENABLE
	SendspinStatus snapshot;
	{
		StatusLock lock;
		snapshot = status;
	}
	Sendspin_PublishMqtt(snapshot);
	#endif

	const bool groupStopped = lastGroupPlaying && !playing && connected;
	lastGroupPlaying = playing;
	Sendspin_CheckSleepModes(groupStopped);

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
	#if defined(NEOPIXEL_ENABLE) && defined(SENDSPIN_ENABLE_VISUALIZER)
VisualizerListener visualizerListener;
	#endif
	#if defined(NEOPIXEL_ENABLE) && defined(SENDSPIN_ENABLE_COLOR)
ColorListener colorListener;
	#endif
ClientListener clientListener;
NetworkProvider networkProvider;
NvsPersistence persistence;

} // namespace

void Sendspin_Init(void) {
	prefsSendspin.begin(kPrefsNamespace, false);
	enabledSetting = prefsSendspin.getBool(kPrefsEnabled, true);
	unpairedAccess = prefsSendspin.getBool("unpaired", false);
	ledMode = prefsSendspin.getUChar("ledMode", static_cast<uint8_t>(SendspinLedMode::Progress));
	statusMutex = xSemaphoreCreateMutex();
	if (statusMutex == nullptr) {
		return;
	}
	{
		StatusLock lock;
		status.enabled = enabledSetting;
		status.unpairedAccess = unpairedAccess;
	}
	if (!enabledSetting) {
		Log_Println("Sendspin: disabled in the settings", LOGLEVEL_NOTICE);
		return;
	}
	if (System_GetOperationMode() != OPMODE_NORMAL) {
		return; // Bluetooth modes switch WiFi off
	}

	ring = xRingbufferCreateWithCaps(kRingBytes, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	sentEvents = xQueueCreate(kQueueLen, sizeof(int64_t));
	writeRecords = xQueueCreate(kQueueLen, sizeof(uint32_t));
	trackCommands = xQueueCreate(4, sizeof(uint8_t));
	if (ring == nullptr || sentEvents == nullptr || writeRecords == nullptr || trackCommands == nullptr
		|| xTaskCreatePinnedToCore(Sendspin_OutputTask, "sendspinOut", kOutputTaskStack, nullptr, kOutputTaskPriority, &outputTask, 1) != pdPASS) {
		Log_Println("Sendspin: out of memory", LOGLEVEL_ERROR);
		return;
	}

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
	// In order of preference: lossless FLAC, then Opus (lossy, a fraction of the bandwidth; 48 kHz only),
	// then uncompressed PCM
	playerConfig.audio_formats = {
		{SendspinCodecFormat::FLAC, 2, 48000, 16},
		{SendspinCodecFormat::FLAC, 2, 44100, 16},
	#ifdef SENDSPIN_ENABLE_OPUS
		{SendspinCodecFormat::OPUS, 2, 48000, 16},
	#endif
		{ SendspinCodecFormat::PCM, 2, 48000, 16},
		{ SendspinCodecFormat::PCM, 2, 44100, 16},
	};
	playerConfig.audio_buffer_capacity = 512 * 1024; // in PSRAM
	PlayerRole &player = client->add_player(std::move(playerConfig));
	player.set_listener(&playerListener);
	playerRole = &player;

	client->add_metadata().set_listener(&metadataListener);
	client->add_controller();
	#if defined(NEOPIXEL_ENABLE) && defined(SENDSPIN_ENABLE_VISUALIZER)
	// Spectrum and beats for the LED ring's spectrum mode. Always requested, so the mode can be switched
	// at runtime; it's a few hundred bytes per second.
	VisualizerRoleConfig visualizerConfig;
	visualizerConfig.support.buffer_capacity = 32 * 1024; // frames wait for their display time like audio
	visualizerConfig.stream.types = {VisualizerDataType::SPECTRUM, VisualizerDataType::BEAT};
	visualizerConfig.stream.rate_max = 30;
	visualizerConfig.stream.spectrum = VisualizerSpectrumConfig {kSendspinSpectrumBins, VisualizerSpectrumScale::MEL, 40, 16000};
	visualizerConfig.display_offset_ms = 20; // the LED task draws every ~20 ms
	client->add_visualizer(std::move(visualizerConfig)).set_listener(&visualizerListener);
	#endif
	#if defined(NEOPIXEL_ENABLE) && defined(SENDSPIN_ENABLE_COLOR)
	client->add_color().set_listener(&colorListener);
	#endif

	client->set_listener(&clientListener);
	client->set_network_provider(&networkProvider);
	client->set_persistence_provider(&persistence);
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
	Sendspin_StopOutput(true);
	client->stop();
	Sendspin_WaitOutputStopped(500);
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
		case FIRSTTRACK:
		case LASTTRACK:
		case NEXTFOLDER:
		case PREVIOUSFOLDER:
			return true; // no equivalent on the server; don't let them wake up the paused local playlist
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
	if (client == nullptr) {
		// not running: nothing to apply, just persist (the NVS API is thread-safe)
		if (enabled != unpairedAccess) {
			prefsSendspin.putBool("unpaired", enabled);
		}
		unpairedAccess = enabled;
		return;
	}
	unpairedAccess = enabled;
	pendingUnpairedAccess = enabled ? 1 : 0;
}

bool Sendspin_GetUnpairedAccess(void) {
	return unpairedAccess;
}

void Sendspin_SetEnabled(bool enabled) {
	if (enabled == enabledSetting) {
		return;
	}
	enabledSetting = enabled;
	prefsSendspin.putBool(kPrefsEnabled, enabled);
	Log_Printf(LOGLEVEL_NOTICE, "Sendspin: %s after the next restart", enabled ? "enabled" : "disabled");
}

bool Sendspin_GetEnabled(void) {
	return enabledSetting;
}

bool Sendspin_LedIsPlaying(void) {
	return ledPlaying;
}

uint8_t Sendspin_LedGetProgress(void) {
	return ledProgress;
}

bool Sendspin_LedGetSpectrum(SendspinSpectrum &spectrum) {
	if (ledMode != static_cast<uint8_t>(SendspinLedMode::Spectrum) || !visualizerStreaming || millis() - spectrumAt > kSpectrumStaleMs) {
		return false;
	}
	for (uint8_t i = 0; i < kSendspinSpectrumBins; i++) {
		spectrum.levels[i] = spectrumLevels[i];
	}
	const uint32_t sinceBeat = millis() - beatAt;
	spectrum.beat = (sinceBeat < kBeatFlashMs) ? 255 * (kBeatFlashMs - sinceBeat) / kBeatFlashMs : 0;
	spectrum.haveColors = haveColors;
	const uint32_t primary = primaryColor;
	const uint32_t accent = accentColor;
	for (uint8_t c = 0; c < 3; c++) {
		spectrum.primary[c] = primary >> (16 - 8 * c);
		spectrum.accent[c] = accent >> (16 - 8 * c);
	}
	return true;
}

void Sendspin_SetLedMode(SendspinLedMode mode) {
	if (static_cast<uint8_t>(mode) == ledMode) {
		return;
	}
	ledMode = static_cast<uint8_t>(mode);
	prefsSendspin.putUChar("ledMode", static_cast<uint8_t>(mode));
}

SendspinLedMode Sendspin_GetLedMode(void) {
	return static_cast<SendspinLedMode>(ledMode.load());
}

uint8_t Sendspin_SleepTracksLeft(void) {
	return isSource ? sleepTracksLeft.load() : 0;
}

void Sendspin_OnMqttConnected(void) {
	#ifdef MQTT_ENABLE
	mqttForce = true;
	#endif
}

#endif
