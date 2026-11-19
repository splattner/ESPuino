#pragma once

#include <Arduino.h>

#include <stdint.h>

// Sendspin receiver (https://www.sendspin-audio.com/): lets a Sendspin server such as Music Assistant
// stream synchronized audio to ESPuino. Runs in normal operation mode next to local playback: when the
// server starts a stream, local playback is paused and the stream takes over the speaker; starting local
// playback again (RFID, web UI, ...) hands the speaker back.

#ifdef SENDSPIN_ENABLE
void Sendspin_Init(void);
void Sendspin_Cyclic(void);
void Sendspin_Exit(void);

// Advertise the receiver via mDNS (_sendspin._tcp). Call after MDNS.begin().
void Sendspin_AdvertiseMdns(void);

// True while the Sendspin stream is the current audio source (streaming or paused by the server).
bool Sendspin_IsSource(void);

// Called by AudioPlayer_SetTrackControl(): while Sendspin is the source, play/pause, next, previous and
// stop are forwarded to the server. Returns true if the command was consumed. Callable from any task.
bool Sendspin_HandleTrackCommand(uint8_t trackCommand);

// Called whenever ESPuino's volume changed (0..AUDIOPLAYER_VOLUME_MAX). Callable from any task.
void Sendspin_OnLocalVolumeChanged(uint8_t volume);

// Snapshot for the web UI. Refreshed by Sendspin_Cyclic(); Sendspin_GetStatus() is callable from any task.
struct SendspinStatus {
	bool enabled = false; // "Sendspin enabled" setting as of boot (changes take effect after a restart)
	bool running = false; // client started (enabled, normal mode, PSRAM)
	bool connected = false;
	bool paired = false; // connection uses a long-term pairing record
	bool playing = false; // the server's group is playing
	bool isSource = false; // Sendspin owns the speaker and the player controls
	bool unpairedAccess = false;
	String serverName;
	String title;
	String artist;
	String album;
	String artworkUrl;
	uint32_t progressMs = 0;
	uint32_t durationMs = 0; // 0: unknown (e.g. radio)
	String clientId;
	String pairingToken;
};
void Sendspin_GetStatus(SendspinStatus &status);

// "Allow unpaired access": servers without a pairing record may play. Persisted; callable from any task.
void Sendspin_SetUnpairedAccess(bool enabled);
bool Sendspin_GetUnpairedAccess(void);

// Sleep modes while streaming ("after this track", "after 5 tracks", "after playlist"): ESPuino counts the
// server's track changes and treats the group stopping as the end of the playlist. The modification
// cards/commands are accepted while Sendspin is the source even without a local playlist.
uint8_t Sendspin_SleepTracksLeft(void); // "after 5 tracks": tracks left while Sendspin is the source, else 0

// MQTT: (re)publish the source, track and Sendspin state topics on the next refresh. Callable from any task.
void Sendspin_OnMqttConnected(void);

// LED ring (callable from any task, meaningful while Sendspin_IsSource())
enum class SendspinLedMode : uint8_t {
	Progress = 0, // track progress like local playback (stream animation for radio)
	Spectrum = 1, // spectrum visualizer, if the server streams visualizer data (progress otherwise)
};
constexpr uint8_t kSendspinSpectrumBins = 24;
bool Sendspin_LedIsPlaying(void); // the server's group is playing
uint8_t Sendspin_LedGetProgress(void); // 0..100; 0xFF: unknown track length (e.g. radio)
// True if spectrum mode is selected and visualizer data is streaming: copies the current band levels
// (0..255, low to high frequency), the beat flash (0..255, decaying) and, if the server sent one, the
// album art's colors (rgb, have* false otherwise)
struct SendspinSpectrum {
	uint8_t levels[kSendspinSpectrumBins];
	uint8_t beat;
	bool haveColors;
	uint8_t primary[3];
	uint8_t accent[3];
};
bool Sendspin_LedGetSpectrum(SendspinSpectrum &spectrum);
void Sendspin_SetLedMode(SendspinLedMode mode); // persisted
SendspinLedMode Sendspin_GetLedMode(void);

// "Sendspin enabled": when off, the receiver isn't started at all (no mDNS, no PSRAM buffers). Persisted;
// takes effect after a restart. Callable from any task.
void Sendspin_SetEnabled(bool enabled);
bool Sendspin_GetEnabled(void);
#endif
