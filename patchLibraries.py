# Applies small source patches to downloaded libraries (.pio/libdeps/<env>/...) before the build.
#
# PlatformIO installs lib_deps before it runs "pre:" scripts, so the files are there when this runs.
# Every patch is idempotent (marker check) and never fails the build: if the library changed and the
# original snippet can't be found, it only prints a warning so the patch can be revisited/dropped.

Import("env")  # noqa: F821 (provided by PlatformIO)

import os

PATCH_MARKER = "ESPUINO-PATCH"

PATCHES = [
    {
        # ESP32-audioI2S 4.0.1: Ogg files (Opus, Vorbis, FLAC-in-Ogg) end early by the size of everything
        # before the first audio page. m_audioDataReadPtr counts from file offset 0 for Ogg, but once the
        # first audio frame is decoded setDecoderItems() sets m_audioDataStart and shrinks m_audioDataSize
        # by it, without rebasing the read pointer. playAudioData() then reports EOF m_audioDataStart bytes
        # too early. Unnoticeable with a few KB of tags, but a big embedded cover (yt-dlp: ~900 KB) cuts an
        # 86 s song at ~23 s. The seek code already treats the pointer as relative to m_audioDataStart
        # (m_audioDataReadPtr = (resumeFilePos + offset) - m_audioDataStart), so rebasing it matches that.
        "name": "audioI2S: rebase read pointer when Ogg audio data start becomes known",
        "file": os.path.join("ESP32-audioI2S", "src", "Audio.cpp"),
        "search": (
            "    if (m_decoder->getAudioDataStart() > 0) { // only flac-ogg, native flac sets audioDataStart in readFlacHeader()\n"
            "        m_audioDataStart = m_decoder->getAudioDataStart();\n"
            "    }\n"
        ),
        "replace": (
            "    if (m_decoder->getAudioDataStart() > 0) { // only flac-ogg, native flac sets audioDataStart in readFlacHeader()\n"
            "        // " + PATCH_MARKER + ": Ogg counted the read pointer from file offset 0 so far; make it relative to the\n"
            "        // audio data start like everywhere else, otherwise EOF is reported m_audioDataStart bytes too early.\n"
            "        const uint32_t oggDataStart = m_decoder->getAudioDataStart();\n"
            "        if (m_audioDataStart == 0 && m_audioDataReadPtr >= oggDataStart) { m_audioDataReadPtr -= oggDataStart; }\n"
            "        m_audioDataStart = oggDataStart;\n"
            "    }\n"
        ),
    },
]


def apply_patches():
    libdeps = os.path.join(env.subst("$PROJECT_LIBDEPS_DIR"), env.subst("$PIOENV"))  # noqa: F821
    for patch in PATCHES:
        path = os.path.join(libdeps, patch["file"])
        if not os.path.isfile(path):
            print(f"patchLibraries: {patch['file']} not found, skipping '{patch['name']}'")
            continue
        with open(path, "r", encoding="utf-8", newline="") as f:
            src = f.read()
        if PATCH_MARKER in src and patch["replace"] in src:
            print(f"patchLibraries: already applied: {patch['name']}")
            continue
        if src.count(patch["search"]) != 1:
            print(f"patchLibraries: WARNING: '{patch['name']}' does not apply (library changed?), skipped")
            continue
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write(src.replace(patch["search"], patch["replace"]))
        print(f"patchLibraries: applied: {patch['name']}")


apply_patches()
