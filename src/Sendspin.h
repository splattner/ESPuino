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
	bool running = false; // client started (normal mode, PSRAM)
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
#endif
