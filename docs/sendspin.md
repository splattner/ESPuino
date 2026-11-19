# Sendspin: stream from Music Assistant to ESPuino

[Sendspin](https://www.sendspin-audio.com/) is an open protocol for synchronized multi-room audio.
With it, a Sendspin server such as [Music Assistant](https://www.music-assistant.io/) can play on
ESPuino like on any other speaker: pick ESPuino as the player in Music Assistant, start a playlist,
radio station or album there, and ESPuino plays it, in sync with other Sendspin speakers in the same
group.

Sendspin runs alongside normal operation. Local playback (RFID cards, the web interface) and
Sendspin take turns on the speaker:

- When the server starts a stream, local playback is paused and the stream takes over.
- When local playback starts again (a card, the web interface, the play button with a local
  playlist), ESPuino fades the stream out and leaves the Sendspin group. The rest of the group keeps
  playing.

## Requirements

- A board with **PSRAM** and **16 MB flash** (the `lolin_d32_pro`, `lolin_d32_pro_sdmmc_pe` and
  `complete` builds). On other boards `SENDSPIN_ENABLE` is switched off automatically: the stream is
  buffered in PSRAM, and Sendspin adds about 400 KB of firmware, more than the 4 MB and 8 MB
  partition layouts have room for.
- Normal operation mode. Sendspin isn't available in the Bluetooth modes.
- A Sendspin server on the same network, e.g. Music Assistant with its Sendspin player provider.

## Setup

1. Build and flash as usual. `SENDSPIN_ENABLE` is on by default in `settings.h`.
2. ESPuino announces itself via mDNS (`_sendspin._tcp`, port 8928). Music Assistant finds it on its
   own and lists it under the ESPuino's hostname.
3. **Pair it.** Open the **Sendspin** tab of ESPuino's web interface, copy the pairing token
   (`SP:...`) and enter it in Music Assistant, in the settings of the ESPuino player. The status in
   the Sendspin tab changes to "Connected to ... (paired)".

Anyone who knows the pairing token can pair a server, so only give it to servers you trust. As an
alternative to pairing, **Allow unpaired access** in the Sendspin tab lets every Sendspin server on
the network play without pairing. Leave it off unless you trust everyone on your network.

To switch Sendspin off without reflashing, untick **Enable Sendspin receiver** in the Sendspin tab
and restart ESPuino. It then doesn't start at all: no mDNS announcement, no memory used.

## Using it

| On ESPuino | While a Sendspin stream plays |
|---|---|
| Play/pause, next, previous, stop (buttons, web interface, MQTT `trackcontrol`) | Sent to the server, i.e. they control the Music Assistant queue. First/last track and next/previous folder have no equivalent and are ignored. |
| Volume (rotary encoder, buttons, web interface, MQTT `loudness`) | Synchronized both ways with the volume of the player in Music Assistant. ESPuino's volume curve and maximum volume (speaker/headphones) still apply. |
| RFID card, playing a file from the web interface | Leaves the Sendspin group and plays locally. |
| Sleep timer in minutes | Works as usual; ESPuino fades out and disconnects from the server before it turns off. |
| Sleep after this track / after 5 tracks / after playlist | ESPuino counts the server's track changes; "after playlist" (and the other two, at the latest) triggers when the server stops playing. A track change or stop caused by ESPuino's own buttons within three seconds doesn't count. |
| Mono playback | Applies to the stream too. |

The web interface's player shows the stream's title, artist and cover art. The Sendspin tab shows
the connection, the pairing token and what's playing.

### LED ring

While streaming, the LED ring shows the track progress like for local files; for radio and other
streams without a known length it shows the web radio animation, and when the server pauses, the
pause animation.

Alternatively, **LED ring while streaming: Spectrum** in the Sendspin tab turns the ring into a
spectrum display: each LED shows a frequency band (low to high along the ring) and beats flash the
whole ring. The colors come from the album art if the server sends them, otherwise from the
progress hue settings. This needs a server that sends visualizer data; without it, the ring shows
the progress as above.

### MQTT

While a stream plays, the existing `track` and `pauseplay` topics describe the stream
("Artist - Title", `play`/`pause`). Two more state topics:

| Topic | Payload |
|---|---|
| `source` | `local` or `sendspin` |
| `sendspin_state` | JSON: `{"connected":true,"playing":true,"server":"Music Assistant","title":"...","artist":"...","album":"...","artworkUrl":"http://...","durationMs":215000,"progressMs":42000}`. Sent on every change, and every 5 seconds while playing to update `progressMs`. |

## Technical notes

- Codecs: FLAC and PCM (44.1 and 48 kHz, 16 bit stereo), plus Opus (48 kHz). The server picks one.
- The stream plays on a second I2S port on the same pins as local playback; the audio library keeps
  I2S0 the whole time. After a stream the pins are routed back to I2S0.
- Pairing keys and settings are kept in their own NVS namespace, `sendspin`.
- Implemented with [sendspin-cpp](https://components.espressif.com/components/sendspin/sendspin-cpp),
  `src/Sendspin.cpp`. Its features are selected via `CONFIG_SENDSPIN_ENABLE_*` in
  `sdkconfig.defaults`.
