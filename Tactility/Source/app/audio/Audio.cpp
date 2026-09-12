// Local Audio Player.
//
// Phase 3: app skeleton + volume control.
// Phase 4: MP3 playback via esp_audio_codec.
// Phase 5: music mode -- folder scan of the picked file's parent directory,
//          natural sort of the resulting MP3 list, prev/next transport,
//          autoplay next at EOF, last-played persistence across reboots.
// Phase 6: audiobook mode -- .taudio.json sidecar with last position + chapters,
//          kind auto-detect (path-based + folder heuristic), byte-offset resume,
//          Chapter -/+ jumps, +/-30 s skips.
// Phase 7: live chapter editor -- Edit toggle reveals Mark + -5/-1/+1/+5 s nudge,
//          chapters re-sorted and renumbered on every edit, saved immediately.
// Phase 8: recursive library index -- worker-thread scan of the picked file's
//          root, music/audiobook heuristics, durations frozen into
//          library-index.json (no bulk sidecar writes).
//
// Design:
//
//   UI thread (LVGL)                 Playback thread (FreeRTOS)
//   ----------------                 --------------------------
//   Pick MP3, tap buttons     -->    atomics: pending / skip*
//                                     v
//                                    runPlaylist() loop:
//                                      runFileOnce(playlist[idx])
//                                       ends with reason:
//                                         EOF/Stop/Skip/Shutdown
//                                      advance idx based on reason
//                                     v
//   updates label via lv_timer <--   nowPlaying string (mutex)
//
// All audio_stream_write() calls happen on the playback thread. The playback
// thread NEVER touches lv_obj -- it only publishes state that a periodic
// lv_timer on the UI thread reads.

#include "lvgl/lvgl.h"

#include <Tactility/app/fileselection/FileSelection.h>
#include <Tactility/file/File.h>
#include <Tactility/DeprecatedPaths.h>
#include <Tactility/service/audio/Audio.h>

#include <app/event.h>
#include <app/manager.h>
#include <app/manifest.h>
#include <app/scheduler.h>
#include <app/stream.h>

#include <lvgl_window_manager/window_manager.h>
#include <lvgl/widgets/toolbar.h>
#include <lvgl/widgets/sliderbox.h>
#include <lvgl.h>

#include <tactility/check.h>
#include <tactility/device.h>
#include <tactility/drivers/audio_stream.h>
#include <tactility/log.h>

#ifdef ESP_PLATFORM
#include <esp_audio_dec_default.h>
#include <esp_audio_simple_dec.h>
#include <esp_audio_simple_dec_default.h>
#endif

#include <cJSON.h>

// Portable FreeRTOS shims (map to freertos/... on ESP32, plain ... on POSIX).
#include <tactility/freertos/freertos.h>
#include <tactility/freertos/queue.h>
#include <tactility/freertos/task.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <cstring>
#include <sys/stat.h>
#include <vector>

namespace tt::app::audio {

constexpr auto* TAG = "AudioApp";
// Reverse-domain app id (vendor-prefixed) per project convention for new apps;
// also determines the per-app user-data directory via tt::getAppUserPath().
constexpr auto* APP_ID = "one.tactility.audio";
// Pre-rename id, kept only for one-time data migration.
constexpr auto* LEGACY_APP_ID = "Audio";
constexpr auto* LAST_PLAYED_FILE = "lastplayed.txt";
constexpr auto* LIBRARY_INDEX_FILE = "library-index.json";
// Subdirectory of the app user-data dir holding per-track resume/chapter files.
constexpr auto* RESUME_TRACKING_DIR = "resume-tracking";
constexpr size_t MAX_INDEX_TRACKS = 512;
constexpr size_t MAX_INDEX_DIRS = 256;

// UI request from the LVGL thread to the playback thread.
enum class PlaybackRequest : uint8_t {
    None,
    PlayCurrent, // play playlist[playlistIndex]; reads latest playlist snapshot
    Stop,
};

// Reason a single-file playback ended, drives the playlist loop's next step.
enum class PlaybackReason : uint8_t {
    ReachedEnd,
    Stopped,
    SkippedNext,
    SkippedPrev,
    Shutdown,
    OpenFailed,
    Seeked, // user requested seek within same file; runPlaylist re-runs with startBytes
};

// Sidecar chapter entry.
struct Chapter {
    int number = 1;
    std::string title;
    int64_t start_ms = 0;
};

struct LibraryIndexEntry {
    std::string path;
    std::string kind;
    std::string title;
    int64_t size_bytes = 0;
    int64_t duration_ms = 0;
};

// In-memory mirror of the `.taudio.json` sidecar.
struct SidecarData {
    std::string audio_file;
    std::string kind;              // "audiobook" | "music" | "audio"
    std::string title;
    std::string author;
    // Resume tracking is opt-in: default-on for detected audiobooks, off for
    // music/other MP3s so plain music never spawns tracking files.
    bool tracking_enabled = false;
    int64_t duration_ms = 0;
    int64_t last_position_ms = 0;
    // Extension over the base schema so seeking is exact regardless of MP3 bitrate.
    int64_t last_position_bytes = 0;
    std::vector<Chapter> chapters;
};

struct PlaybackState {
    // Producer/consumer signalling.
    std::atomic<PlaybackRequest> pending{PlaybackRequest::None};
    std::atomic<bool> shutdown{false};
    std::atomic<bool> skipNext{false};
    std::atomic<bool> skipPrev{false};
    std::atomic<bool> stopRequested{false};
    std::atomic<bool> autoplay{true};
    std::atomic<bool> playing{false};
    // Pause is implemented by stopping the active run and resuming from the
    // captured byte/ms position. Holding the output stream open while not
    // writing can repeat the final DMA buffer on this hardware.
    std::atomic<bool> paused{false};

    // Playlist snapshot updated by UI thread and consumed by playback thread.
    // playlist is a snapshot so we can iterate without holding the mutex through
    // long-running writes; playlistIndex is an atomic cursor into it.
    std::mutex playlistMutex;
    std::vector<std::string> playlist;
    std::atomic<int> playlistIndex{-1};

    // Read by lv_timer on the UI thread; written by playback thread.
    std::mutex nowPlayingMutex;
    std::string nowPlayingPath;
    std::atomic<bool> nowPlayingDirty{true};

    // Position / seek plumbing (Phase 6).
    // currentPositionMs is decoded PCM millis + any startBytes-implied offset.
    // currentFileBytes is the current fread offset in the source file.
    // currentFileSize is the file size in bytes, or 0 if unknown.
    // seekRequestMs >= 0 requests a seek within the current file; playback
    // task consumes it by returning PlaybackReason::Seeked so runPlaylist can
    // restart the same track at the new position.
    std::atomic<int64_t> currentPositionMs{0};
    std::atomic<int64_t> currentFileBytes{0};
    std::atomic<int64_t> currentFileSize{0};
    // Bytes-per-ms for the input MP3, learned from decoded audio. Used to
    // translate a millisecond seek target into a byte offset.
    std::atomic<int64_t> bytesPerMsX1000{0}; // fixed-point *1000; 0 = unknown
    std::atomic<int64_t> seekRequestMs{-1};
    // runPlaylist reads this when handling Seeked to fseek the reopened file.
    std::atomic<int64_t> nextStartBytes{0};
    std::atomic<int64_t> nextStartMs{0};

    TaskHandle_t task = nullptr;
};

namespace {

// ---- Natural sort ----------------------------------------------------------

// Case-insensitive natural comparison so "Book 2" sorts before "Book 10".
bool natLess(const std::string& a, const std::string& b) {
    size_t i = 0, j = 0;
    while (i < a.size() && j < b.size()) {
        bool aIsDigit = std::isdigit(static_cast<unsigned char>(a[i])) != 0;
        bool bIsDigit = std::isdigit(static_cast<unsigned char>(b[j])) != 0;
        if (aIsDigit && bIsDigit) {
            // Compare numeric runs by value, not lexicographically.
            unsigned long long av = 0, bv = 0;
            while (i < a.size() && std::isdigit(static_cast<unsigned char>(a[i]))) {
                av = av * 10 + static_cast<unsigned long long>(a[i++] - '0');
            }
            while (j < b.size() && std::isdigit(static_cast<unsigned char>(b[j]))) {
                bv = bv * 10 + static_cast<unsigned long long>(b[j++] - '0');
            }
            if (av != bv) return av < bv;
        } else {
            char ac = static_cast<char>(std::tolower(static_cast<unsigned char>(a[i])));
            char bc = static_cast<char>(std::tolower(static_cast<unsigned char>(b[j])));
            if (ac != bc) return ac < bc;
            i++;
            j++;
        }
    }
    return a.size() < b.size();
}

// ---- Sidecar + kind detection (Phase 6) ------------------------------------
//
// Sidecars live in the app's private user-data directory (same pattern as
// other Tactility apps) so the user's media folders stay clean. The filename
// is the MP3 path with '/' and ' ' flattened to '_'. A one-time migration
// reads/removes the legacy next-to-file `.taudio.json` if present.

std::string legacySidecarPathFor(const std::string& mp3Path) {
    if (mp3Path.size() < 4) return mp3Path + ".taudio.json";
    return mp3Path.substr(0, mp3Path.size() - 4) + ".taudio.json";
}

std::string sidecarPathFor(const std::string& mp3Path, const std::string& sidecarDir) {
    std::string flat = mp3Path;
    // Strip a leading "/sdcard/" so names stay short and stable.
    constexpr std::string_view prefix = "/sdcard/";
    if (flat.rfind(prefix, 0) == 0) flat = flat.substr(prefix.size());
    for (auto& c : flat) {
        if (c == '/' || c == ' ') c = '_';
    }
    if (flat.size() > 4) flat = flat.substr(0, flat.size() - 4); // drop .mp3
    return sidecarDir + "/" + flat + ".taudio.json";
}

// Return "audiobook" if the path lives inside an /Audiobooks/ segment, "music"
// if inside /Music/, else "audio". Case-insensitive on the segment name.
std::string detectKindFromPath(const std::string& mp3Path) {
    std::string lower = mp3Path;
    for (auto& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (lower.find("/audiobooks/") != std::string::npos) return "audiobook";
    if (lower.find("/music/") != std::string::npos) return "music";
    return "audio";
}

bool isSupportedKind(const std::string& kind) {
    return kind == "audiobook" || kind == "music" || kind == "audio";
}

void normalizeChapters(std::vector<Chapter>& chapters) {
    if (chapters.empty()) {
        chapters.push_back({.number = 1, .title = "Chapter 1", .start_ms = 0});
    }
    for (auto& chapter : chapters) {
        if (chapter.start_ms < 0) chapter.start_ms = 0;
    }
    std::sort(chapters.begin(), chapters.end(),
              [](const Chapter& a, const Chapter& b) { return a.start_ms < b.start_ms; });
    if (chapters.front().start_ms != 0) {
        chapters.insert(chapters.begin(), {.number = 1, .title = "Chapter 1", .start_ms = 0});
    }
    // Remove exact duplicates after clamping/sorting.
    chapters.erase(std::unique(chapters.begin(), chapters.end(),
                               [](const Chapter& a, const Chapter& b) {
                                   return a.start_ms == b.start_ms;
                               }),
                   chapters.end());
    for (size_t i = 0; i < chapters.size(); ++i) {
        chapters[i].number = static_cast<int>(i) + 1;
        chapters[i].title = "Chapter " + std::to_string(i + 1);
    }
}

// Read the .taudio.json sidecar for a file if present. Missing/corrupt sidecar
// yields a default SidecarData with kind derived from the path. `sidecarPath`
// is the explicit file to read (new-style or legacy).
SidecarData loadSidecar(const std::string& mp3Path, const std::string& sidecarPath) {
    SidecarData data;
    // Basename of the audio file.
    auto slash = mp3Path.find_last_of('/');
    data.audio_file = (slash == std::string::npos) ? mp3Path : mp3Path.substr(slash + 1);
    // Title default: filename without extension.
    if (data.audio_file.size() > 4 &&
        (data.audio_file.substr(data.audio_file.size() - 4) == ".mp3" ||
         data.audio_file.substr(data.audio_file.size() - 4) == ".MP3")) {
        data.title = data.audio_file.substr(0, data.audio_file.size() - 4);
    } else {
        data.title = data.audio_file;
    }
    data.kind = detectKindFromPath(mp3Path);
    data.tracking_enabled = (data.kind == "audiobook");
    data.chapters.push_back({.number = 1, .title = "Chapter 1", .start_ms = 0});

    if (sidecarPath.empty() || !file::isFile(sidecarPath)) {
        normalizeChapters(data.chapters);
        return data;
    }

    std::string body;
    {
        FILE* fp = fopen(sidecarPath.c_str(), "rb");
        if (fp == nullptr) return data;
        char buf[512];
        size_t got;
        while ((got = fread(buf, 1, sizeof(buf), fp)) > 0) body.append(buf, got);
        fclose(fp);
    }

    cJSON* root = cJSON_Parse(body.c_str());
    if (root == nullptr) {
        LOG_W(TAG, "Sidecar %s is not valid JSON; ignoring", sidecarPath.c_str());
        return data;
    }

    auto readString = [&](const char* key, std::string& out) {
        auto* item = cJSON_GetObjectItem(root, key);
        if (item != nullptr && cJSON_IsString(item) && item->valuestring != nullptr) {
            out = item->valuestring;
        }
    };
    auto readNumber = [&](const char* key, int64_t& out) {
        auto* item = cJSON_GetObjectItem(root, key);
        if (item != nullptr && cJSON_IsNumber(item)) {
            out = static_cast<int64_t>(item->valuedouble);
        }
    };
    auto readBool = [&](const char* key, bool& out) {
        auto* item = cJSON_GetObjectItem(root, key);
        if (item != nullptr && cJSON_IsBool(item)) {
            out = cJSON_IsTrue(item);
        }
    };

    readString("audio_file", data.audio_file);
    readString("kind", data.kind);
    readString("title", data.title);
    readString("author", data.author);
    readBool("tracking_enabled", data.tracking_enabled);
    readNumber("duration_ms", data.duration_ms);
    readNumber("last_position_ms", data.last_position_ms);
    readNumber("last_position_bytes", data.last_position_bytes);
    if (!isSupportedKind(data.kind)) data.kind = detectKindFromPath(mp3Path);
    if (data.duration_ms < 0) data.duration_ms = 0;
    if (data.last_position_ms < 0) data.last_position_ms = 0;
    if (data.last_position_bytes < 0) data.last_position_bytes = 0;

    auto* chapters = cJSON_GetObjectItem(root, "chapters");
    if (chapters != nullptr && cJSON_IsArray(chapters)) {
        std::vector<Chapter> parsed;
        int n = cJSON_GetArraySize(chapters);
        for (int i = 0; i < n; ++i) {
            auto* c = cJSON_GetArrayItem(chapters, i);
            if (c == nullptr) continue;
            Chapter ch;
            auto* num = cJSON_GetObjectItem(c, "number");
            auto* ti = cJSON_GetObjectItem(c, "title");
            auto* st = cJSON_GetObjectItem(c, "start_ms");
            if (num != nullptr && cJSON_IsNumber(num)) ch.number = num->valueint;
            if (ti != nullptr && cJSON_IsString(ti) && ti->valuestring != nullptr) ch.title = ti->valuestring;
            if (st != nullptr && cJSON_IsNumber(st)) ch.start_ms = static_cast<int64_t>(st->valuedouble);
            parsed.push_back(ch);
        }
        if (!parsed.empty()) {
            data.chapters = std::move(parsed);
        }
    }

    cJSON_Delete(root);
    normalizeChapters(data.chapters);
    return data;
}

bool saveSidecar(const std::string& sidecarPath, const SidecarData& data) {
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr) return false;
    cJSON_AddStringToObject(root, "schema", "tactility-audio-v1");
    cJSON_AddStringToObject(root, "audio_file", data.audio_file.c_str());
    cJSON_AddStringToObject(root, "kind", data.kind.c_str());
    cJSON_AddStringToObject(root, "title", data.title.c_str());
    cJSON_AddStringToObject(root, "author", data.author.c_str());
    cJSON_AddBoolToObject(root, "tracking_enabled", data.tracking_enabled);
    cJSON_AddNumberToObject(root, "duration_ms", static_cast<double>(data.duration_ms));
    cJSON_AddNumberToObject(root, "last_position_ms", static_cast<double>(data.last_position_ms));
    cJSON_AddNumberToObject(root, "last_position_bytes", static_cast<double>(data.last_position_bytes));
    cJSON* chapters = cJSON_AddArrayToObject(root, "chapters");
    if (chapters != nullptr) {
        for (const auto& c : data.chapters) {
            cJSON* item = cJSON_CreateObject();
            if (item == nullptr) continue;
            cJSON_AddNumberToObject(item, "number", c.number);
            cJSON_AddStringToObject(item, "title", c.title.c_str());
            cJSON_AddNumberToObject(item, "start_ms", static_cast<double>(c.start_ms));
            cJSON_AddItemToArray(chapters, item);
        }
    }
    char* rendered = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (rendered == nullptr) return false;

    bool ok = false;
    {
        FILE* fp = fopen(sidecarPath.c_str(), "wb");
        if (fp != nullptr) {
            size_t len = strlen(rendered);
            ok = (fwrite(rendered, 1, len, fp) == len);
            fclose(fp);
        }
    }
    free(rendered);
    if (!ok) LOG_E(TAG, "saveSidecar: write failed for %s", sidecarPath.c_str());
    return ok;
}

// Forward decls: the worker runs folder scans so the LVGL thread never does.
std::string dirnameOf(const std::string& path);
int64_t fileSizeOf(const std::string& path);
int64_t estimateMp3DurationMs(const std::string& path, int64_t fileSize);
std::vector<std::string> scanFolderForMp3(const std::string& folder);
std::vector<LibraryIndexEntry> scanLibraryForMp3(const std::string& root);
bool saveLibraryIndex(const std::string& indexPath,
                      const std::string& root,
                      const std::vector<LibraryIndexEntry>& entries);

// ---- Sidecar worker (keeps SD/flash I/O off the LVGL thread) ---------------
//
// The UI thread enqueues jobs; this low-priority task performs the actual
// fopen/fread/fwrite/cJSON work. Blocking the LVGL thread with storage I/O
// stalls display refreshes and causes visible partial-render artifacts.

struct SidecarJob {
    enum class Type : uint8_t { InitStorage, Load, Save, AdoptPlaylist, SaveLastPlayed, ScanIndex, Shutdown } type;
    std::string mp3Path {};      // Load / AdoptPlaylist / SaveLastPlayed: track/root path
    std::string sidecarPath {};  // Save: explicit destination
    SidecarData data {};         // Save: snapshot to write
};

struct SidecarWorker {
    std::string appDataDir;
    std::string oldAppDir;
    std::string dir; // app user-data resume-tracking directory (SD card)
    std::string lastPlayedPath;
    std::string legacyLastPlayedPath;
    std::string libraryIndexPath;
    // Previous locations migrated away from: the interim internal-/data dir
    // and the old "sidecars" subdir name under the app dir.
    std::vector<std::string> legacyDirs;
    QueueHandle_t queue = nullptr;
    TaskHandle_t task = nullptr;

    // Result slot for Load jobs, consumed by the UI poll timer.
    std::mutex loadedMutex;
    SidecarData loadedData;
    std::string loadedMp3Path;
    std::atomic<bool> loadedReady{false};

    // Result slot for AdoptPlaylist jobs (folder scan + lastplayed I/O).
    std::mutex playlistReadyMutex;
    std::vector<std::string> playlistReadyFiles;
    int playlistReadyIndex = -1;
    std::string playlistReadyPicked;
    std::atomic<bool> playlistReady{false};

    // Result slot for InitStorage jobs.
    std::mutex lastPlayedMutex;
    std::string lastPlayedReadyPath;
    std::atomic<bool> lastPlayedReady{false};

    // Result slot for ScanIndex jobs.
    std::mutex scanMutex;
    std::string scanRoot;
    size_t scanCount = 0;
    bool scanSaved = false;
    std::atomic<bool> scanReady{false};

    void start() {
        queue = xQueueCreate(8, sizeof(SidecarJob*));
        configASSERT(queue != nullptr);
        BaseType_t ok = xTaskCreate(&SidecarWorker::entry, "audio_sc", 6 * 1024,
                                    this, tskIDLE_PRIORITY + 1, &task);
        if (ok != pdPASS) {
            LOG_E(TAG, "Failed to create sidecar worker task");
            task = nullptr;
        }
    }

    bool enqueue(SidecarJob* job, TickType_t timeout = 0) {
        if (queue == nullptr) {
            delete job;
            return false;
        }
        if (xQueueSend(queue, &job, timeout) != pdTRUE) {
            LOG_W(TAG, "Sidecar queue full; dropping job type=%d", (int) job->type);
            delete job;
            return false;
        }
        return true;
    }

    void requestInitStorage() {
        auto* job = new SidecarJob{.type = SidecarJob::Type::InitStorage};
        enqueue(job, pdMS_TO_TICKS(100));
    }

    void requestLoad(const std::string& mp3Path) {
        auto* job = new SidecarJob{.type = SidecarJob::Type::Load, .mp3Path = mp3Path};
        enqueue(job, pdMS_TO_TICKS(50));
    }

    void requestSave(const std::string& sidecarPath, const SidecarData& data) {
        auto* job = new SidecarJob{.type = SidecarJob::Type::Save,
                                   .sidecarPath = sidecarPath, .data = data};
        enqueue(job);
    }

    void requestAdoptPlaylist(const std::string& pickedPath) {
        auto* job = new SidecarJob{.type = SidecarJob::Type::AdoptPlaylist, .mp3Path = pickedPath};
        enqueue(job, pdMS_TO_TICKS(50));
    }

    void requestSaveLastPlayed(const std::string& path) {
        auto* job = new SidecarJob{.type = SidecarJob::Type::SaveLastPlayed, .mp3Path = path};
        enqueue(job, pdMS_TO_TICKS(50));
    }

    void requestScanIndex(const std::string& rootPath) {
        auto* job = new SidecarJob{.type = SidecarJob::Type::ScanIndex, .mp3Path = rootPath};
        enqueue(job);
    }

    void stop() {
        if (task == nullptr) return;
        auto* job = new SidecarJob{.type = SidecarJob::Type::Shutdown};
        // Block briefly so shutdown is reliably delivered.
        if (queue != nullptr && xQueueSend(queue, &job, pdMS_TO_TICKS(500)) != pdTRUE) {
            delete job;
        }
        for (int i = 0; i < 80 && task != nullptr; ++i) {
            vTaskDelay(pdMS_TO_TICKS(25));
        }
        if (task != nullptr) {
            LOG_W(TAG, "Sidecar worker did not stop before timeout");
            return;
        }
        if (queue != nullptr) {
            vQueueDelete(queue);
            queue = nullptr;
        }
    }

    static void entry(void* param) {
        auto* self = static_cast<SidecarWorker*>(param);
        while (true) {
            SidecarJob* job = nullptr;
            if (xQueueReceive(self->queue, &job, portMAX_DELAY) != pdTRUE || job == nullptr) {
                continue;
            }
            switch (job->type) {
                case SidecarJob::Type::InitStorage: {
                    struct stat st;
                    if (!self->appDataDir.empty() &&
                        stat(self->appDataDir.c_str(), &st) != 0 &&
                        !self->oldAppDir.empty() &&
                        stat(self->oldAppDir.c_str(), &st) == 0) {
                        if (::rename(self->oldAppDir.c_str(), self->appDataDir.c_str()) == 0) {
                            LOG_I(TAG, "Migrated app data dir %s -> %s",
                                  self->oldAppDir.c_str(), self->appDataDir.c_str());
                        }
                    }
                    if (!self->appDataDir.empty()) {
                        file::findOrCreateDirectory(self->appDataDir, 0775);
                    }
                    if (!file::findOrCreateDirectory(self->dir, 0775)) {
                        LOG_E(TAG, "Failed to create resume-tracking dir %s", self->dir.c_str());
                    }

                    std::string statePath = self->lastPlayedPath;
                    bool migrate = false;
                    if (!file::isFile(statePath) && file::isFile(self->legacyLastPlayedPath)) {
                        statePath = self->legacyLastPlayedPath;
                        migrate = true;
                    }
                    std::string loaded;
                    if (file::isFile(statePath)) {
                        FILE* fp = fopen(statePath.c_str(), "r");
                        if (fp != nullptr) {
                            char buf[512];
                            if (fgets(buf, sizeof(buf), fp) != nullptr) {
                                loaded = buf;
                                while (!loaded.empty() && (loaded.back() == '\n' || loaded.back() == '\r')) {
                                    loaded.pop_back();
                                }
                            }
                            fclose(fp);
                        }
                    }
                    if (migrate && !loaded.empty()) {
                        FILE* fp = fopen(self->lastPlayedPath.c_str(), "w");
                        if (fp != nullptr) {
                            fputs(loaded.c_str(), fp);
                            fclose(fp);
                            ::remove(statePath.c_str());
                            LOG_I(TAG, "Migrated lastplayed.txt to %s", self->lastPlayedPath.c_str());
                        }
                    }
                    if (!loaded.empty()) {
                        std::lock_guard lg(self->lastPlayedMutex);
                        self->lastPlayedReadyPath = std::move(loaded);
                        self->lastPlayedReady.store(true);
                    }
                    break;
                }
                case SidecarJob::Type::Load: {
                    std::string path = sidecarPathFor(job->mp3Path, self->dir);
                    bool migrated = false;
                    if (!file::isFile(path)) {
                        // One-time migration: legacy next-to-file location, then
                        // any previous sidecar directory.
                        std::string legacy = legacySidecarPathFor(job->mp3Path);
                        if (file::isFile(legacy)) {
                            path = legacy;
                            migrated = true;
                        } else {
                            for (const auto& legacyDir : self->legacyDirs) {
                                std::string old = sidecarPathFor(job->mp3Path, legacyDir);
                                if (file::isFile(old)) {
                                    path = old;
                                    migrated = true;
                                    break;
                                }
                            }
                        }
                    }
                    SidecarData data = loadSidecar(job->mp3Path, path);
                    if (data.duration_ms <= 0) {
                        data.duration_ms = estimateMp3DurationMs(job->mp3Path, fileSizeOf(job->mp3Path));
                    }
                    if (migrated) {
                        std::string newPath = sidecarPathFor(job->mp3Path, self->dir);
                        if (saveSidecar(newPath, data)) {
                            ::remove(path.c_str());
                            LOG_I(TAG, "Migrated sidecar %s -> %s", path.c_str(), newPath.c_str());
                        }
                    }
                    {
                        std::lock_guard lg(self->loadedMutex);
                        self->loadedData = std::move(data);
                        self->loadedMp3Path = job->mp3Path;
                    }
                    self->loadedReady.store(true);
                    break;
                }
                case SidecarJob::Type::Save:
                    saveSidecar(job->sidecarPath, job->data);
                    break;
                case SidecarJob::Type::SaveLastPlayed: {
                    if (self->lastPlayedPath.empty()) break;
                    FILE* fp = fopen(self->lastPlayedPath.c_str(), "w");
                    if (fp != nullptr) {
                        fputs(job->mp3Path.c_str(), fp);
                        fclose(fp);
                    }
                    break;
                }
                case SidecarJob::Type::AdoptPlaylist: {
                    // Folder listing is SD I/O; never do this on the LVGL thread.
                    std::string picked = job->mp3Path;
                    auto files = scanFolderForMp3(dirnameOf(picked));
                    int idx = -1;
                    for (size_t i = 0; i < files.size(); ++i) {
                        if (files[i] == picked) {
                            idx = static_cast<int>(i);
                            break;
                        }
                    }
                    if (idx < 0 && !files.empty()) idx = 0;
                    {
                        std::lock_guard lg(self->playlistReadyMutex);
                        self->playlistReadyFiles = std::move(files);
                        self->playlistReadyIndex = idx;
                        self->playlistReadyPicked = std::move(picked);
                    }
                    self->playlistReady.store(true);
                    break;
                }
                case SidecarJob::Type::ScanIndex: {
                    std::string root = job->mp3Path;
                    auto entries = scanLibraryForMp3(root);
                    bool saved = saveLibraryIndex(self->libraryIndexPath, root, entries);
                    size_t count = entries.size();
                    {
                        std::lock_guard lg(self->scanMutex);
                        self->scanRoot = std::move(root);
                        self->scanCount = count;
                        self->scanSaved = saved;
                    }
                    self->scanReady.store(true);
                    break;
                }
                case SidecarJob::Type::Shutdown:
                    delete job;
                    self->task = nullptr;
                    vTaskDelete(nullptr);
                    return;
            }
            delete job;
        }
    }
};

// Format "H:MM:SS" or "M:SS".
std::string formatDurationMs(int64_t ms) {
    if (ms < 0) ms = 0;
    int64_t s = ms / 1000;
    int64_t h = s / 3600;
    int64_t m = (s % 3600) / 60;
    int64_t sec = s % 60;
    char buf[32];
    if (h > 0) snprintf(buf, sizeof(buf), "%lld:%02lld:%02lld", (long long) h, (long long) m, (long long) sec);
    else snprintf(buf, sizeof(buf), "%lld:%02lld", (long long) m, (long long) sec);
    return buf;
}

// ---- MP3 file/name helpers -------------------------------------------------

bool hasMp3Extension(const std::string& name) {
    if (name.size() < 4) return false;
    auto ext = name.substr(name.size() - 4);
    for (auto& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext == ".mp3";
}

std::string dirnameOf(const std::string& path) {
    auto slash = path.find_last_of('/');
    if (slash == std::string::npos || slash == 0) return "/";
    return path.substr(0, slash);
}

int64_t fileSizeOf(const std::string& path) {
    struct stat st;
    if (stat(path.c_str(), &st) != 0) return 0;
    return static_cast<int64_t>(st.st_size);
}

int64_t id3v2TagSize(const uint8_t* header, size_t size) {
    if (size < 10 || header[0] != 'I' || header[1] != 'D' || header[2] != '3') return 0;
    int64_t tagSize =
        ((header[6] & 0x7f) << 21) |
        ((header[7] & 0x7f) << 14) |
        ((header[8] & 0x7f) << 7) |
        (header[9] & 0x7f);
    if ((header[5] & 0x10) != 0) tagSize += 10; // footer present
    return tagSize + 10;
}

int mp3BitrateKbpsFromHeader(uint32_t header) {
    if ((header & 0xffe00000u) != 0xffe00000u) return 0;
    int version = (header >> 19) & 0x3; // 3=MPEG1, 2=MPEG2, 0=MPEG2.5
    int layer = (header >> 17) & 0x3;   // 1=Layer III, 2=Layer II, 3=Layer I
    int bitrateIndex = (header >> 12) & 0xf;
    int sampleIndex = (header >> 10) & 0x3;
    if (version == 1 || layer == 0 || bitrateIndex == 0 || bitrateIndex == 0xf || sampleIndex == 0x3) {
        return 0;
    }

    static constexpr int mpeg1Layer1[16] = {0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 0};
    static constexpr int mpeg1Layer2[16] = {0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, 0};
    static constexpr int mpeg1Layer3[16] = {0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0};
    static constexpr int mpeg2Layer1[16] = {0, 32, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256, 0};
    static constexpr int mpeg2Layer23[16] = {0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 0};

    if (version == 3) {
        if (layer == 3) return mpeg1Layer1[bitrateIndex];
        if (layer == 2) return mpeg1Layer2[bitrateIndex];
        return mpeg1Layer3[bitrateIndex];
    }
    if (layer == 3) return mpeg2Layer1[bitrateIndex];
    return mpeg2Layer23[bitrateIndex];
}

int64_t estimateMp3DurationMs(const std::string& path, int64_t fileSize) {
    if (fileSize <= 0) return 0;

    int64_t audioStart = 0;
    uint8_t scan[2048];
    size_t got = 0;
    {
        FILE* fp = fopen(path.c_str(), "rb");
        if (fp == nullptr) return (fileSize * 8) / 128; // conservative 128 kbps fallback
        got = fread(scan, 1, sizeof(scan), fp);
        audioStart = id3v2TagSize(scan, got);
        if (audioStart > 0 && audioStart < fileSize) {
            fseek(fp, static_cast<long>(audioStart), SEEK_SET);
            got = fread(scan, 1, sizeof(scan), fp);
        }
        fclose(fp);
    }

    int bitrateKbps = 0;
    for (size_t i = 0; i + 3 < got; ++i) {
        uint32_t header =
            (static_cast<uint32_t>(scan[i]) << 24) |
            (static_cast<uint32_t>(scan[i + 1]) << 16) |
            (static_cast<uint32_t>(scan[i + 2]) << 8) |
            static_cast<uint32_t>(scan[i + 3]);
        bitrateKbps = mp3BitrateKbpsFromHeader(header);
        if (bitrateKbps > 0) {
            audioStart += static_cast<int64_t>(i);
            break;
        }
    }
    if (bitrateKbps <= 0) bitrateKbps = 128;
    int64_t audioBytes = std::max<int64_t>(1, fileSize - audioStart);
    return (audioBytes * 8) / bitrateKbps;
}

// Scan `folder` for MP3 files and return a naturally-sorted list of absolute paths.
std::vector<std::string> scanFolderForMp3(const std::string& folder) {
    std::vector<std::string> out;
    int seen = 0;
    file::listDirectory(folder, [&out, &folder, &seen](const dirent& entry) {
        seen++;
        std::string name = entry.d_name;
        // Some FAT VFS backends leave d_type=0 (DT_UNKNOWN) even for regular files;
        // rely on the extension check rather than d_type.
        if (name == "." || name == "..") return;
        if (hasMp3Extension(name)) {
            out.push_back(file::getChildPath(folder, name));
        }
    });
    LOG_I(TAG, "scanFolderForMp3: saw %d entries, %zu are MP3", seen, out.size());
    std::sort(out.begin(), out.end(), [](const std::string& a, const std::string& b) {
        return natLess(file::getLastPathSegment(a), file::getLastPathSegment(b));
    });
    return out;
}

std::string detectKindForIndex(const std::string& path, int64_t sizeBytes) {
    std::string kind = detectKindFromPath(path);
    if (kind != "audio") return kind;
    // Duration is not available without decoding. File size is a cheap proxy:
    // long single-file books are commonly tens of MB; keep normal short tracks
    // as "audio" unless the path says Music/Audiobooks explicitly.
    constexpr int64_t audiobookSizeFloor = 32LL * 1024LL * 1024LL;
    return sizeBytes >= audiobookSizeFloor ? "audiobook" : "audio";
}

std::vector<LibraryIndexEntry> scanLibraryForMp3(const std::string& root) {
    std::vector<LibraryIndexEntry> entries;
    std::vector<std::string> dirs;
    dirs.push_back(root);

    for (size_t dirIndex = 0; dirIndex < dirs.size(); ++dirIndex) {
        if (dirIndex >= MAX_INDEX_DIRS || entries.size() >= MAX_INDEX_TRACKS) break;
        const std::string folder = dirs[dirIndex];
        file::listDirectory(folder, [&entries, &dirs, &folder](const dirent& entry) {
            if (entries.size() >= MAX_INDEX_TRACKS || dirs.size() >= MAX_INDEX_DIRS) return;
            std::string name = entry.d_name;
            if (name == "." || name == "..") return;
            std::string path = file::getChildPath(folder, name);
            bool isDir = entry.d_type == file::TT_DT_DIR || entry.d_type == file::TT_DT_CHR;
            // FAT can report unknown d_type. Only check directories for
            // non-MP3 names, and only on the worker thread.
            if (!isDir && !hasMp3Extension(name)) {
                isDir = file::isDirectory(path);
            }
            if (isDir) {
                dirs.push_back(std::move(path));
            } else if (hasMp3Extension(name)) {
                LibraryIndexEntry indexed;
                indexed.path = path;
                indexed.title = file::getLastPathSegment(path);
                indexed.size_bytes = fileSizeOf(path);
                indexed.duration_ms = estimateMp3DurationMs(path, indexed.size_bytes);
                indexed.kind = detectKindForIndex(path, indexed.size_bytes);
                entries.push_back(std::move(indexed));
            }
        });
    }

    std::sort(entries.begin(), entries.end(), [](const LibraryIndexEntry& a, const LibraryIndexEntry& b) {
        if (a.kind != b.kind) return a.kind < b.kind;
        std::string aFolder = dirnameOf(a.path);
        std::string bFolder = dirnameOf(b.path);
        if (aFolder != bFolder) return natLess(aFolder, bFolder);
        return natLess(a.title, b.title);
    });
    return entries;
}

bool saveLibraryIndex(const std::string& indexPath,
                      const std::string& root,
                      const std::vector<LibraryIndexEntry>& entries) {
    if (indexPath.empty()) return false;
    cJSON* doc = cJSON_CreateObject();
    if (doc == nullptr) return false;
    cJSON_AddStringToObject(doc, "schema", "tactility-audio-index-v1");
    cJSON_AddStringToObject(doc, "root", root.c_str());
    cJSON_AddNumberToObject(doc, "count", static_cast<double>(entries.size()));
    cJSON* tracks = cJSON_AddArrayToObject(doc, "tracks");
    for (const auto& entry : entries) {
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "path", entry.path.c_str());
        cJSON_AddStringToObject(item, "kind", entry.kind.c_str());
        cJSON_AddStringToObject(item, "title", entry.title.c_str());
        cJSON_AddNumberToObject(item, "size_bytes", static_cast<double>(entry.size_bytes));
        cJSON_AddNumberToObject(item, "duration_ms", static_cast<double>(entry.duration_ms));
        cJSON_AddItemToArray(tracks, item);
    }
    char* rendered = cJSON_PrintUnformatted(doc);
    cJSON_Delete(doc);
    if (rendered == nullptr) return false;

    bool ok = false;
    {
        FILE* fp = fopen(indexPath.c_str(), "wb");
        if (fp != nullptr) {
            size_t len = strlen(rendered);
            ok = fwrite(rendered, 1, len, fp) == len;
            fclose(fp);
        }
    }
    free(rendered);
    if (!ok) LOG_E(TAG, "saveLibraryIndex: write failed for %s", indexPath.c_str());
    return ok;
}

// ---- Decoder registration --------------------------------------------------

std::once_flag g_decoderRegisterOnce;

void ensureDecodersRegistered() {
#ifdef ESP_PLATFORM
    std::call_once(g_decoderRegisterOnce, [] {
        esp_audio_err_t derr = esp_audio_dec_register_default();
        if (derr != ESP_AUDIO_ERR_OK) {
            LOG_E(TAG, "esp_audio_dec_register_default failed: %d", (int) derr);
        }
        esp_audio_err_t serr = esp_audio_simple_dec_register_default();
        if (serr != ESP_AUDIO_ERR_OK) {
            LOG_E(TAG, "esp_audio_simple_dec_register_default failed: %d", (int) serr);
        }
    });
#endif
}

Device* findAudioStreamDevice() {
    Device* result = nullptr;
    device_for_each_of_type(&AUDIO_STREAM_TYPE, &result, [](Device* device, void* ctx) -> bool {
        if (device_is_ready(device)) {
            *static_cast<Device**>(ctx) = device;
            return false;
        }
        return true;
    });
    return result;
}

void publishNowPlaying(PlaybackState* state, const std::string& path) {
    {
        std::lock_guard lg(state->nowPlayingMutex);
        state->nowPlayingPath = path;
    }
    state->nowPlayingDirty.store(true);
}

// ---- One-file playback -----------------------------------------------------

// Play one MP3 file end-to-end. Returns the reason the run finished so the
// outer playlist loop can decide whether to advance, hold, or exit.
// If startBytes > 0, the file is fseek'd to that offset before decoding begins
// (MP3 decoders resync at the next valid frame). startMs is the wall-clock
// position that byte offset represents, used to keep currentPositionMs accurate.
PlaybackReason runFileOnce(PlaybackState* state, const std::string& path,
                           int64_t startBytes, int64_t startMs) {
#ifndef ESP_PLATFORM
    // Simulator: no decoder/audio hardware. Pretend the file opened and is
    // idling so the UI can be exercised.
    LOG_W(TAG, "Simulator build: not actually playing %s", path.c_str());
    state->playing.store(true);
    publishNowPlaying(state, path);
    while (!state->shutdown.load() && !state->stopRequested.load() &&
           !state->skipNext.load() && !state->skipPrev.load() &&
           state->seekRequestMs.load() < 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    state->playing.store(false);
    if (state->shutdown.load()) return PlaybackReason::Shutdown;
    if (state->skipNext.exchange(false)) return PlaybackReason::SkippedNext;
    if (state->skipPrev.exchange(false)) return PlaybackReason::SkippedPrev;
    if (state->seekRequestMs.exchange(-1) >= 0) return PlaybackReason::Seeked;
    return PlaybackReason::Stopped;
#else
    ensureDecodersRegistered();

    Device* streamDevice = findAudioStreamDevice();
    if (streamDevice == nullptr) {
        LOG_E(TAG, "No audio-stream device found");
        return PlaybackReason::OpenFailed;
    }

    if (!hasMp3Extension(path)) {
        LOG_E(TAG, "Unsupported audio format (mp3 only): %s", path.c_str());
        return PlaybackReason::OpenFailed;
    }

    FILE* fp = nullptr;
    int64_t fileSize = 0;
    {
        fp = fopen(path.c_str(), "rb");
        if (fp != nullptr) {
            if (fseek(fp, 0, SEEK_END) == 0) {
                long sz = ftell(fp);
                if (sz >= 0) fileSize = sz;
            }
            fseek(fp, 0, SEEK_SET);
            if (startBytes > 0 && startBytes < fileSize) {
                fseek(fp, (long) startBytes, SEEK_SET);
            } else {
                startBytes = 0;
                startMs = 0;
            }
        }
    }
    if (fp == nullptr) {
        LOG_E(TAG, "Failed to open file: %s", path.c_str());
        return PlaybackReason::OpenFailed;
    }
    state->currentFileSize.store(fileSize);
    state->currentFileBytes.store(startBytes);
    state->currentPositionMs.store(startMs);

    esp_audio_simple_dec_cfg_t decCfg = {
        .dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_MP3,
        .dec_cfg = nullptr,
        .cfg_size = 0,
        .use_frame_dec = false,
    };
    esp_audio_simple_dec_handle_t dec = nullptr;
    esp_audio_err_t derr = esp_audio_simple_dec_open(&decCfg, &dec);
    if (derr != ESP_AUDIO_ERR_OK) {
        LOG_E(TAG, "esp_audio_simple_dec_open failed: %d", (int) derr);
        {
            fclose(fp);
        }
        return PlaybackReason::OpenFailed;
    }

    constexpr size_t inChunkSize = 4096;
    std::vector<uint8_t> inBuf(inChunkSize);
    std::vector<uint8_t> outBuf(1152 * 2 * sizeof(int16_t) * 2);

    AudioStreamHandle handle = nullptr;
    AudioStreamConfig openedCfg = {};
    uint32_t bytesPerPcmSec = 0;
    bool eos = false;
    size_t inFill = 0;
    size_t inOffset = 0;
    size_t totalPcmBytes = 0;
    int64_t fileOffset = startBytes;
    PlaybackReason reason = PlaybackReason::ReachedEnd;

    state->playing.store(true);
    publishNowPlaying(state, path);
    LOG_I(TAG, "Play start: %s (startBytes=%lld startMs=%lld size=%lld)",
          path.c_str(), (long long) startBytes, (long long) startMs, (long long) fileSize);

    while (true) {
        if (state->shutdown.load()) { reason = PlaybackReason::Shutdown; break; }
        if (state->stopRequested.load()) { reason = PlaybackReason::Stopped; break; }
        if (state->skipNext.exchange(false)) { reason = PlaybackReason::SkippedNext; break; }
        if (state->skipPrev.exchange(false)) { reason = PlaybackReason::SkippedPrev; break; }
        if (state->seekRequestMs.load() >= 0) { reason = PlaybackReason::Seeked; break; }

        // Pause: hold position without decoding or writing. Skip/seek/stop
        // remain responsive while paused.
        while (state->paused.load() &&
               !state->shutdown.load() && !state->stopRequested.load() &&
               !state->skipNext.load() && !state->skipPrev.load() &&
               state->seekRequestMs.load() < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        if (!eos && inFill - inOffset < inChunkSize / 2) {
            size_t remaining = inFill - inOffset;
            if (inOffset > 0 && remaining > 0) {
                memmove(inBuf.data(), inBuf.data() + inOffset, remaining);
            }
            inOffset = 0;
            inFill = remaining;

            size_t want = inChunkSize - inFill;
            size_t got = 0;
            {
                got = fread(inBuf.data() + inFill, 1, want, fp);
            }
            inFill += got;
            fileOffset += static_cast<int64_t>(got);
            state->currentFileBytes.store(fileOffset);
            if (got == 0) {
                eos = true;
            }
        }

        if (eos && inOffset >= inFill) {
            break; // natural end of file, reason stays ReachedEnd
        }

        esp_audio_simple_dec_raw_t raw = {};
        raw.buffer = inBuf.data() + inOffset;
        raw.len = static_cast<uint32_t>(inFill - inOffset);
        raw.eos = eos;
        raw.consumed = 0;

        esp_audio_simple_dec_out_t out = {};
        out.buffer = outBuf.data();
        out.len = static_cast<uint32_t>(outBuf.size());

        derr = esp_audio_simple_dec_process(dec, &raw, &out);
        if (derr == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
            uint32_t needed = out.needed_size > 0 ? out.needed_size : (uint32_t) outBuf.size() * 2;
            outBuf.resize(needed);
            continue;
        }
        if (derr != ESP_AUDIO_ERR_OK) {
            LOG_E(TAG, "esp_audio_simple_dec_process failed: %d", (int) derr);
            break;
        }
        inOffset += raw.consumed;

        if (out.decoded_size > 0) {
            if (handle == nullptr) {
                esp_audio_simple_dec_info_t info = {};
                if (esp_audio_simple_dec_get_info(dec, &info) != ESP_AUDIO_ERR_OK) {
                    LOG_E(TAG, "Decoder info not ready after first decoded frame");
                    break;
                }
                openedCfg.sample_rate = info.sample_rate;
                openedCfg.bits_per_sample = info.bits_per_sample ? info.bits_per_sample : 16;
                openedCfg.channels = info.channel;
                bytesPerPcmSec = openedCfg.sample_rate * openedCfg.channels * (openedCfg.bits_per_sample / 8);
                LOG_I(TAG, "Opening output: %u Hz, %u ch, %u bit",
                      (unsigned) openedCfg.sample_rate, (unsigned) openedCfg.channels, (unsigned) openedCfg.bits_per_sample);
                error_t oerr = audio_stream_open_output(streamDevice, &openedCfg, &handle);
                if (oerr != ERROR_NONE) {
                    LOG_E(TAG, "audio_stream_open_output failed: %d", oerr);
                    handle = nullptr;
                    break;
                }
            }

            size_t bytesWritten = 0;
            error_t werr;
            do {
                werr = audio_stream_write(handle, out.buffer, out.decoded_size, &bytesWritten, pdMS_TO_TICKS(250));
            } while (werr == ERROR_TIMEOUT &&
                     !state->stopRequested.load() &&
                     !state->shutdown.load() &&
                     !state->skipNext.load() &&
                     !state->skipPrev.load() &&
                     state->seekRequestMs.load() < 0);
            if (werr != ERROR_NONE) {
                if (werr != ERROR_TIMEOUT) {
                    LOG_E(TAG, "audio_stream_write failed: %d", werr);
                }
                break;
            }
            totalPcmBytes += bytesWritten;

            // Update playback position from decoded PCM count.
            if (bytesPerPcmSec > 0) {
                int64_t sessionMs = (int64_t) totalPcmBytes * 1000 / (int64_t) bytesPerPcmSec;
                int64_t positionMs = startMs + sessionMs;
                state->currentPositionMs.store(positionMs);
                // Learn bytes-per-ms of the *input* MP3 from played-so-far data.
                // Store as fixed-point *1000 to preserve fractional accuracy.
                if (positionMs > 500 && fileOffset > startBytes) {
                    int64_t bpMsX1000 = ((fileOffset - startBytes) * 1000) / (positionMs - startMs);
                    state->bytesPerMsX1000.store(bpMsX1000);
                }
            }
        }
    }

    if (handle != nullptr) {
        audio_stream_close(handle);
    }
    esp_audio_simple_dec_close(dec);
    {
        fclose(fp);
    }

    state->playing.store(false);
    LOG_I(TAG, "Play end (%zu PCM bytes, reason=%d)", totalPcmBytes, (int) reason);
    return reason;
#endif // ESP_PLATFORM
}

// ---- Playlist loop ---------------------------------------------------------

void runPlaylist(PlaybackState* state) {
    // First-run startBytes/startMs come from nextStartBytes/nextStartMs (set by
    // the UI before requesting PlayCurrent so an audiobook can resume from its
    // last known position). Subsequent files start at 0.
    int64_t startBytes = state->nextStartBytes.exchange(0);
    int64_t startMs = state->nextStartMs.exchange(0);

    while (true) {
        if (state->shutdown.load()) break;
        if (state->stopRequested.load()) {
            state->stopRequested.store(false);
            break;
        }

        std::string path;
        {
            std::lock_guard lg(state->playlistMutex);
            int idx = state->playlistIndex.load();
            if (idx < 0 || idx >= static_cast<int>(state->playlist.size())) {
                break; // playlist exhausted
            }
            path = state->playlist[idx];
        }

        PlaybackReason reason = runFileOnce(state, path, startBytes, startMs);
        // Default: fresh start for whatever plays next.
        startBytes = 0;
        startMs = 0;
        if (reason != PlaybackReason::Stopped) {
            state->paused.store(false);
        }

        int nextIdx = state->playlistIndex.load();
        int size = 0;
        {
            std::lock_guard lg(state->playlistMutex);
            size = static_cast<int>(state->playlist.size());
        }

        switch (reason) {
            case PlaybackReason::Shutdown:
                return;
            case PlaybackReason::Stopped:
                state->stopRequested.store(false);
                if (!state->paused.load()) {
                    publishNowPlaying(state, "");
                }
                return;
            case PlaybackReason::SkippedNext:
                nextIdx = (nextIdx + 1 < size) ? nextIdx + 1 : -1;
                break;
            case PlaybackReason::SkippedPrev:
                nextIdx = (nextIdx > 0) ? nextIdx - 1 : 0;
                break;
            case PlaybackReason::ReachedEnd:
                if (state->autoplay.load() && nextIdx + 1 < size) {
                    nextIdx += 1;
                } else {
                    publishNowPlaying(state, "");
                    return;
                }
                break;
            case PlaybackReason::OpenFailed:
                // Skip forward past a broken file if in autoplay mode; otherwise stop.
                if (state->autoplay.load() && nextIdx + 1 < size) {
                    nextIdx += 1;
                } else {
                    publishNowPlaying(state, "");
                    return;
                }
                break;
            case PlaybackReason::Seeked: {
                // Translate the requested ms into a byte offset via the learned
                // bytes-per-ms and restart the SAME track from there.
                int64_t targetMs = state->seekRequestMs.exchange(-1);
                if (targetMs < 0) targetMs = 0;
                int64_t bpMsX1000 = state->bytesPerMsX1000.load();
                int64_t fileSize = state->currentFileSize.load();
                int64_t targetBytes = 0;
                if (bpMsX1000 > 0) {
                    targetBytes = (targetMs * bpMsX1000) / 1000;
                } else if (fileSize > 0 && state->currentPositionMs.load() > 0) {
                    // Fallback: linear proportion using current known ratio.
                    targetBytes = (fileSize * targetMs) / std::max<int64_t>(1, state->currentPositionMs.load());
                }
                if (fileSize > 0 && targetBytes >= fileSize) targetBytes = fileSize - 1024;
                if (targetBytes < 0) targetBytes = 0;
                startBytes = targetBytes;
                startMs = targetMs;
                // nextIdx stays the same: same file re-runs with seek offset.
                break;
            }
        }

        state->playlistIndex.store(nextIdx);
        if (nextIdx < 0) {
            publishNowPlaying(state, "");
            return;
        }
    }
}

// ---- Task entry ------------------------------------------------------------

void playbackTaskEntry(void* param) {
    auto* state = static_cast<PlaybackState*>(param);

    while (!state->shutdown.load()) {
        auto request = state->pending.exchange(PlaybackRequest::None);

        switch (request) {
            case PlaybackRequest::PlayCurrent:
                state->stopRequested.store(false);
                state->skipNext.store(false);
                state->skipPrev.store(false);
                runPlaylist(state);
                break;
            case PlaybackRequest::Stop:
            case PlaybackRequest::None:
            default:
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
        }
    }

    state->task = nullptr;
    vTaskDelete(nullptr);
}

} // namespace

// ---- Current-architecture app plumbing -------------------------------------
//
// The old class-based lifecycle (AudioApp with onCreate/onShow/onHide/onResult/
// onStop) maps onto the current app-module model as follows:
//
//   onCreate  -> initAppData(), called from appMain() before the window exists
//   onShow    -> createWidgets(), invoked by window_manager_create_ext() on the
//                LVGL thread; runs again whenever this window resurfaces as
//                topmost (e.g. after the file picker closes)
//   onHide    -> destroyWidgets(), invoked when this window's widgets are about
//                to be deleted while the app keeps running (e.g. the file picker
//                window was created on top)
//   onResult  -> APP_EVENT_RESULT handling in appMain()'s event loop; the
//                picked path arrives through the AppStream bound to the
//                FileSelection child app
//   onDestroy -> appMain()'s epilogue after APP_EVENT_CLOSE
//
// Everything below the plumbing (playback engine, sidecar worker, playlist
// logic, persistence cadence) is unchanged from the original implementation.

namespace {

struct Context {
    uint32_t appInstanceId = 0;
    TaskEventGroup* eventGroup = nullptr;

    PlaybackState playback;

    // Last-played path persisted to user data so the app resumes on the same track
    // (not playing automatically, just pre-selected) after a reboot.
    std::string lastPlayedFilePath;
    std::string savedLastPath;
    std::string appDataDir; // tt::getAppUserPath(APP_ID), resolved in initAppData
    std::string libraryRootPath;

    // Sidecar state for the currently-selected track. Kept in sync between
    // pick-file, playback progress polling, and sidecar auto-save. All actual
    // file I/O happens on sidecarWorker so the LVGL thread never blocks on
    // storage (which would stall display refreshes and garble rendering).
    std::mutex sidecarMutex;
    SidecarData currentSidecar;
    std::string currentSidecarPath;    // MP3 path the sidecar is for; empty = none loaded
    std::string sidecarLoadPendingPath; // MP3 path a Load job was requested for
    int64_t lastSavedPositionMs = -1;
    uint32_t saveCounterTicks = 0;
    SidecarWorker sidecarWorker;
    int64_t lastRenderedPositionSec = -1; // throttle position label redraws

    lv_obj_t* statusLabel = nullptr;
    lv_obj_t* nowPlayingLabel = nullptr; // title, top of screen
    lv_obj_t* infoLabel = nullptr;       // "Audiobook - Track 1 of 2"
    lv_obj_t* progressBar = nullptr;     // chapter-relative (audiobook) / track (music)
    lv_obj_t* progressLabel = nullptr;   // "M:SS / M:SS"
    lv_obj_t* playPauseLabel = nullptr;  // symbol inside the big play/pause button
    lv_obj_t* edgeLeftLabel = nullptr;   // "Ch <<"/prev-track, mode-dependent
    lv_obj_t* edgeRightLabel = nullptr;  // "Ch >>"/next-track, mode-dependent
    // Only the switch whose state is driven by non-UI events needs to be kept:
    // a loaded sidecar (or Mark Chapter) can turn resume tracking on by itself.
    // The autoplay/edit switches are only ever read from their own event target.
    lv_obj_t* trackingSwitch = nullptr;  // per-track resume tracking on/off
    lv_obj_t* editorRow = nullptr;       // Mark/nudge buttons, hidden unless edit mode
    int lastEditChapterIdx = -1;         // chapter targeted by nudge (guarded by sidecarMutex)
    bool lastPlayPauseWasPause = false;  // throttle symbol redraws
    lv_timer_t* pollTimer = nullptr;

    // Volume slider. The slider edits the global output volume -- the same value
    // the headset buttons change -- so its on-screen position must track external
    // changes too. lastVolumeUserMs suppresses that sync briefly after a user edit
    // so an in-progress drag is never fought.
    lv_obj_t* volumeSliderBox = nullptr;
    uint32_t lastVolumeUserMs = 0;

    uint32_t fileSelectionLaunchId = 0;
    // Result plumbing for the FileSelection child app (picked file path).
    AppStream fileResultStream {};
    uint8_t fileResultBuffer[512] {};

    // Guards ensureBackendStarted(): the worker and playback tasks start on
    // first use, not in initAppData. Only touched from the LVGL thread.
    bool backendStarted = false;
};

// Forward declarations (the old class declared all of these as members, so
// definition order did not matter; free functions need these).
void applyReadyLastPlayedIfAny(Context* self);
void applyReadyPlaylistIfAny(Context* self);
void applyReadyScanIfAny(Context* self);
void loadSidecarForCurrentTrack(Context* self);

// Adopt a completed sidecar Load job if it matches the track we asked for.
void adoptLoadedSidecarIfReady(Context* self) {
    if (!self->sidecarWorker.loadedReady.load()) return;
    std::lock_guard lg(self->sidecarMutex);
    if (self->sidecarLoadPendingPath.empty()) return;
    std::string loadedPath;
    SidecarData data;
    {
        std::lock_guard lg2(self->sidecarWorker.loadedMutex);
        if (self->sidecarWorker.loadedMp3Path != self->sidecarLoadPendingPath) return;
        loadedPath = self->sidecarWorker.loadedMp3Path;
        data = std::move(self->sidecarWorker.loadedData);
    }
    self->sidecarWorker.loadedReady.store(false);
    self->currentSidecar = std::move(data);
    self->currentSidecarPath = loadedPath;
    self->sidecarLoadPendingPath.clear();
    self->lastSavedPositionMs = -1;
    self->lastRenderedPositionSec = -1;
    self->lastEditChapterIdx = -1;
    if (self->trackingSwitch != nullptr) {
        if (self->currentSidecar.tracking_enabled) {
            lv_obj_add_state(self->trackingSwitch, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(self->trackingSwitch, LV_STATE_CHECKED);
        }
    }
    // Seed the position display from the saved position only when idle;
    // during active playback the decoder owns the position counter.
    if (!self->playback.playing.load()) {
        self->playback.currentPositionMs.store(
            self->currentSidecar.last_position_ms > 0 ? self->currentSidecar.last_position_ms : 0);
        self->playback.currentFileBytes.store(self->currentSidecar.last_position_bytes);
    }
    self->playback.nowPlayingDirty.store(true);
}

// Total duration is frozen when the sidecar is loaded or Rescan writes the
// index. Do not fall back to live bytes/ms learning here, because that made
// the displayed total move during playback.
int64_t estimateDurationMsLocked(const Context* self) {
    // Call with sidecarMutex held.
    return self->currentSidecar.duration_ms > 0 ? self->currentSidecar.duration_ms : 0;
}

// Progress is chapter-relative for audiobooks (elapsed within the current
// chapter vs chapter length) and track-relative for music.
void updateProgressUi(Context* self) {
    if (self->progressLabel == nullptr) return;
    int64_t pos = self->playback.currentPositionMs.load();
    if (pos / 1000 == self->lastRenderedPositionSec) return;
    self->lastRenderedPositionSec = pos / 1000;

    int64_t shownPos = pos;
    int64_t shownTotal = 0;
    {
        std::lock_guard lg(self->sidecarMutex);
        int64_t duration = estimateDurationMsLocked(self);
        shownTotal = duration;
        if (self->currentSidecar.kind == "audiobook" && !self->currentSidecar.chapters.empty()) {
            const auto& chapters = self->currentSidecar.chapters;
            int64_t chStart = 0;
            int64_t chEnd = duration;
            for (size_t i = 0; i < chapters.size(); ++i) {
                if (chapters[i].start_ms <= pos) {
                    chStart = chapters[i].start_ms;
                    chEnd = (i + 1 < chapters.size()) ? chapters[i + 1].start_ms : duration;
                } else {
                    break;
                }
            }
            shownPos = pos - chStart;
            if (shownPos < 0) shownPos = 0;
            shownTotal = chEnd - chStart;
        }
    }

    std::string text = formatDurationMs(shownPos);
    if (shownTotal > 0) text += " / " + formatDurationMs(shownTotal);
    lv_label_set_text(self->progressLabel, text.c_str());
    if (self->progressBar != nullptr && shownTotal > 0 && shownPos <= shownTotal) {
        lv_bar_set_value(self->progressBar, static_cast<int32_t>((shownPos * 1000) / shownTotal), LV_ANIM_OFF);
    } else if (self->progressBar != nullptr) {
        lv_bar_set_value(self->progressBar, 0, LV_ANIM_OFF);
    }
}

// Called from lv_timer (UI thread) to reflect playback state into the labels.
void refreshFromPlaybackState(Context* self) {
    applyReadyLastPlayedIfAny(self);
    applyReadyPlaylistIfAny(self);
    applyReadyScanIfAny(self);
    adoptLoadedSidecarIfReady(self);

    // Play/pause symbol follows live state (cheap; only redrawn on change).
    bool showPause = self->playback.playing.load() && !self->playback.paused.load();
    if (self->playPauseLabel != nullptr && showPause != self->lastPlayPauseWasPause) {
        self->lastPlayPauseWasPause = showPause;
        lv_label_set_text(self->playPauseLabel, showPause ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
    }

    if (self->playback.nowPlayingDirty.exchange(false)) {
        std::string path;
        {
            std::lock_guard lg(self->playback.nowPlayingMutex);
            path = self->playback.nowPlayingPath;
        }
        if (self->nowPlayingLabel != nullptr) {
            if (path.empty()) {
                lv_label_set_text(self->nowPlayingLabel, "(nothing playing)");
            } else {
                lv_label_set_text(self->nowPlayingLabel, file::getLastPathSegment(path).c_str());
            }
        }
        // If the now-playing track changed under us (playlist advanced on
        // the playback thread), request a sidecar load for the new track.
        if (!path.empty()) {
            bool needLoad = false;
            {
                std::lock_guard lg(self->sidecarMutex);
                needLoad = (path != self->currentSidecarPath && path != self->sidecarLoadPendingPath);
            }
            if (needLoad) {
                {
                    std::lock_guard lg(self->sidecarMutex);
                    self->sidecarLoadPendingPath = path;
                }
                self->sidecarWorker.requestLoad(path);
            }
        }
        // Info line: kind + playlist position.
        if (self->infoLabel != nullptr) {
            std::string kind;
            {
                std::lock_guard lg(self->sidecarMutex);
                kind = self->currentSidecar.kind;
            }
            if (kind.empty()) kind = "audio";
            kind[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(kind[0])));
            std::string info = kind;
            {
                std::lock_guard lg(self->playback.playlistMutex);
                int idx = self->playback.playlistIndex.load();
                size_t total = self->playback.playlist.size();
                if (total > 0 && idx >= 0) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), " - %d of %zu", idx + 1, total);
                    info += buf;
                }
            }
            lv_label_set_text(self->infoLabel, info.c_str());
        }
        // Edge buttons: chapter jumps for audiobooks, track skip for music.
        bool audiobookMode = false;
        {
            std::lock_guard lg(self->sidecarMutex);
            audiobookMode = (self->currentSidecar.kind == "audiobook");
        }
        if (self->edgeLeftLabel != nullptr && self->edgeRightLabel != nullptr) {
            if (audiobookMode) {
                lv_label_set_text(self->edgeLeftLabel, LV_SYMBOL_LEFT " Ch");
                lv_label_set_text(self->edgeRightLabel, "Ch " LV_SYMBOL_RIGHT);
            } else {
                lv_label_set_text(self->edgeLeftLabel, LV_SYMBOL_PREV);
                lv_label_set_text(self->edgeRightLabel, LV_SYMBOL_NEXT);
            }
        }
    }
    updateProgressUi(self);
}

// Autosave the sidecar with the latest position; called from the poll timer
// every ~20 seconds while playback is active. The write itself happens on
// the sidecar worker thread.
void maybeSaveSidecar(Context* self) {
    if (!self->playback.playing.load()) return;
    int64_t pos = self->playback.currentPositionMs.load();
    int64_t bytes = self->playback.currentFileBytes.load();
    if (pos <= 0) return;
    {
        std::lock_guard lg(self->sidecarMutex);
        if (self->currentSidecarPath.empty()) return;
        if (!self->currentSidecar.tracking_enabled) return;
        if (self->lastSavedPositionMs >= 0 && std::abs(pos - self->lastSavedPositionMs) < 15000) return;
        self->currentSidecar.last_position_ms = pos;
        self->currentSidecar.last_position_bytes = bytes;
        self->lastSavedPositionMs = pos;
        self->sidecarWorker.requestSave(sidecarPathFor(self->currentSidecarPath, self->sidecarWorker.dir),
                                        self->currentSidecar);
    }
}

// Defined below; shared by the on-screen button and the media Play/Pause key.
void handlePlayPauseToggle(Context* self);

void pollTimerCb(lv_timer_t* timer) {
    auto* self = static_cast<Context*>(lv_timer_get_user_data(timer));
    // Headset/media Play/Pause button: the service latch is set from the HID pump task,
    // so act on it here (LVGL thread) where the playback toggle is safe to run.
    if (service::audio::consumePlayPauseRequest()) {
        handlePlayPauseToggle(self);
    }
    refreshFromPlaybackState(self);
    // Track external volume changes (e.g. headset buttons) on the slider.
    // lvgl_sliderbox_set_value does not fire the value-changed callback, so this
    // cannot feed back into setOutputVolume; the grace period after a user edit
    // keeps a drag or rapid +/- taps from being overridden.
    if (self->volumeSliderBox != nullptr && lv_tick_elaps(self->lastVolumeUserMs) > 1500) {
        auto globalVolume = static_cast<int32_t>(service::audio::getOutputVolume());
        if (globalVolume != lvgl_sliderbox_get_value(self->volumeSliderBox)) {
            lvgl_sliderbox_set_value(self->volumeSliderBox, globalVolume, LV_ANIM_OFF);
        }
    }
    // Tick every 500ms; save every ~20 seconds of ticks while playing.
    self->saveCounterTicks++;
    if (self->saveCounterTicks >= 40) {
        self->saveCounterTicks = 0;
        maybeSaveSidecar(self);
    }
}

// Force-save the sidecar regardless of the debounce interval, used on hide/close
// so we don't lose progress just because we're a few seconds shy of the next
// scheduled autosave. Enqueued to the worker; the queue is FIFO so a following
// Shutdown job drains after this write completes.
void maybeSaveSidecarNow(Context* self) {
    int64_t pos = self->playback.currentPositionMs.load();
    int64_t bytes = self->playback.currentFileBytes.load();
    if (pos <= 0) return;
    {
        std::lock_guard lg(self->sidecarMutex);
        if (self->currentSidecarPath.empty()) return;
        if (!self->currentSidecar.tracking_enabled) return;
        self->currentSidecar.last_position_ms = pos;
        self->currentSidecar.last_position_bytes = bytes;
        self->lastSavedPositionMs = pos;
        self->sidecarWorker.requestSave(sidecarPathFor(self->currentSidecarPath, self->sidecarWorker.dir),
                                        self->currentSidecar);
    }
}

// Common seek entry: sets seekRequestMs so the playback loop returns Seeked
// and runPlaylist restarts the current file at the new byte offset.
void requestSeekMs(Context* self, int64_t targetMs) {
    if (targetMs < 0) targetMs = 0;
    if (!self->playback.playing.load()) {
        // Not playing: prime nextStartBytes for the next PlayCurrent.
        int64_t bpMsX1000 = self->playback.bytesPerMsX1000.load();
        int64_t startBytes = 0;
        if (bpMsX1000 > 0) startBytes = (targetMs * bpMsX1000) / 1000;
        self->playback.nextStartMs.store(targetMs);
        self->playback.nextStartBytes.store(startBytes);
        return;
    }
    self->playback.seekRequestMs.store(targetMs);
}

void onBack30Cb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    int64_t pos = self->playback.currentPositionMs.load();
    requestSeekMs(self, pos - 30000);
}

void onFwd30Cb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    int64_t pos = self->playback.currentPositionMs.load();
    requestSeekMs(self, pos + 30000);
}

// Chapter editor: mark the current position and nudge the selected marker.
// Chapters live in the sidecar, so marking implies resume tracking for
// the track. Every change is saved immediately.

void saveSidecarSnapshotLocked(Context* self) {
    // Call with sidecarMutex held.
    self->sidecarWorker.requestSave(sidecarPathFor(self->currentSidecarPath, self->sidecarWorker.dir),
                                    self->currentSidecar);
}

void markChapter(Context* self) {
    int64_t pos = self->playback.currentPositionMs.load();
    {
        std::lock_guard lg(self->sidecarMutex);
        if (self->currentSidecarPath.empty()) {
            lv_label_set_text(self->statusLabel, "Pick a track first.");
            return;
        }
        auto& chapters = self->currentSidecar.chapters;
        // Duplicate prevention: a chapter within 2 s of this position is
        // selected for nudging instead of creating a near-duplicate.
        for (size_t i = 0; i < chapters.size(); ++i) {
            if (std::abs(chapters[i].start_ms - pos) < 2000) {
                self->lastEditChapterIdx = static_cast<int>(i);
                std::string msg = chapters[i].title + " already here; nudge to adjust.";
                lv_label_set_text(self->statusLabel, msg.c_str());
                return;
            }
        }
        Chapter chapter;
        chapter.start_ms = pos;
        chapters.push_back(chapter);
        normalizeChapters(chapters);
        for (size_t i = 0; i < chapters.size(); ++i) {
            if (chapters[i].start_ms == pos) {
                self->lastEditChapterIdx = static_cast<int>(i);
                break;
            }
        }
        self->currentSidecar.tracking_enabled = true; // chapters require the file
        saveSidecarSnapshotLocked(self);
        std::string msg = "Marked " + chapters[self->lastEditChapterIdx].title +
                          " at " + formatDurationMs(pos);
        lv_label_set_text(self->statusLabel, msg.c_str());
    }
    if (self->trackingSwitch != nullptr) {
        lv_obj_add_state(self->trackingSwitch, LV_STATE_CHECKED);
    }
}

void nudgeChapter(Context* self, int64_t deltaMs) {
    {
        std::lock_guard lg(self->sidecarMutex);
        if (self->currentSidecarPath.empty() || self->lastEditChapterIdx < 0 ||
            self->lastEditChapterIdx >= static_cast<int>(self->currentSidecar.chapters.size())) {
            lv_label_set_text(self->statusLabel, "Mark a chapter first.");
            return;
        }
        auto& chapters = self->currentSidecar.chapters;
        int64_t newStart = chapters[self->lastEditChapterIdx].start_ms + deltaMs;
        if (newStart < 0) newStart = 0;
        chapters[self->lastEditChapterIdx].start_ms = newStart;
        normalizeChapters(chapters);
        for (size_t i = 0; i < chapters.size(); ++i) {
            if (chapters[i].start_ms == newStart) {
                self->lastEditChapterIdx = static_cast<int>(i);
                break;
            }
        }
        saveSidecarSnapshotLocked(self);
        std::string msg = chapters[self->lastEditChapterIdx].title +
                          " -> " + formatDurationMs(newStart);
        lv_label_set_text(self->statusLabel, msg.c_str());
    }
}

void onMarkChapterCb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    markChapter(self);
}

void onNudgeBack5Cb(lv_event_t* e) {
    nudgeChapter(static_cast<Context*>(lv_event_get_user_data(e)), -5000);
}
void onNudgeBack1Cb(lv_event_t* e) {
    nudgeChapter(static_cast<Context*>(lv_event_get_user_data(e)), -1000);
}
void onNudgeFwd1Cb(lv_event_t* e) {
    nudgeChapter(static_cast<Context*>(lv_event_get_user_data(e)), 1000);
}
void onNudgeFwd5Cb(lv_event_t* e) {
    nudgeChapter(static_cast<Context*>(lv_event_get_user_data(e)), 5000);
}

void onEditModeSwitchCb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    auto* sw = lv_event_get_target_obj(e);
    bool editMode = lv_obj_has_state(sw, LV_STATE_CHECKED);
    if (self->editorRow != nullptr) {
        if (editMode) {
            lv_obj_remove_flag(self->editorRow, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(self->editorRow, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void onPickFileCb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    self->fileSelectionLaunchId = fileselection::startForExistingFile(
        self->appInstanceId, self->fileResultStream, self->fileResultBuffer,
        sizeof(self->fileResultBuffer), self->eventGroup);
}

void requestLibraryRescan(Context* self) {
    if (self->playback.playing.load() || self->playback.paused.load()) {
        lv_label_set_text(self->statusLabel, "Stop playback before Rescan.");
        return;
    }
    if (self->libraryRootPath.empty()) {
        lv_label_set_text(self->statusLabel, "Pick an MP3 first.");
        return;
    }
    self->sidecarWorker.requestScanIndex(self->libraryRootPath);
    lv_label_set_text(self->statusLabel, "Scanning library...");
}

void onRescanCb(lv_event_t* e) {
    requestLibraryRescan(static_cast<Context*>(lv_event_get_user_data(e)));
}

// Start playback of the current playlist cursor, resuming from the saved
// position when resume tracking is enabled for the track.
void startPlayback(Context* self) {
    int idx = -1;
    size_t total = 0;
    std::string firstPath;
    {
        std::lock_guard lg(self->playback.playlistMutex);
        idx = self->playback.playlistIndex.load();
        total = self->playback.playlist.size();
        if (idx >= 0 && idx < static_cast<int>(total)) firstPath = self->playback.playlist[idx];
    }
    LOG_I(TAG, "Play tapped: playlist size=%zu idx=%d first=%s",
          total, idx, firstPath.c_str());
    if (total == 0 || idx < 0 || idx >= static_cast<int>(total)) {
        lv_label_set_text(self->statusLabel, "Pick an MP3 first.");
        return;
    }
    // Resume starts 1 s before the saved position so the listener re-hears
    // the last moment of context; the byte offset is scaled proportionally.
    int64_t resumeBytes = 0;
    int64_t resumeMs = 0;
    {
        std::lock_guard lg(self->sidecarMutex);
        if (self->currentSidecar.tracking_enabled &&
            self->currentSidecar.last_position_bytes > 0) {
            resumeMs = self->currentSidecar.last_position_ms;
            resumeBytes = self->currentSidecar.last_position_bytes;
            if (resumeMs > 1000) {
                int64_t adjustedMs = resumeMs - 1000;
                resumeBytes = (resumeBytes * adjustedMs) / resumeMs;
                resumeMs = adjustedMs;
            } else {
                resumeMs = 0;
                resumeBytes = 0;
            }
        }
    }
    self->playback.paused.store(false);
    self->playback.nextStartBytes.store(resumeBytes);
    self->playback.nextStartMs.store(resumeMs);
    self->playback.stopRequested.store(true); // stop any current run first
    self->playback.pending.store(PlaybackRequest::PlayCurrent);
    lv_label_set_text(self->statusLabel,
                      resumeBytes > 0 ? "Resuming..." : "Playing...");
}

void pausePlayback(Context* self) {
    maybeSaveSidecarNow(self);
    self->playback.paused.store(true);
    self->playback.stopRequested.store(true);
    lv_label_set_text(self->statusLabel, "Pausing...");
}

void resumePausedPlayback(Context* self) {
    int64_t resumeMs = self->playback.currentPositionMs.load();
    int64_t resumeBytes = self->playback.currentFileBytes.load();
    if (resumeMs > 1000 && resumeBytes > 0) {
        int64_t adjustedMs = resumeMs - 1000;
        resumeBytes = (resumeBytes * adjustedMs) / resumeMs;
        resumeMs = adjustedMs;
    } else {
        resumeMs = 0;
        resumeBytes = 0;
    }
    self->playback.nextStartBytes.store(resumeBytes);
    self->playback.nextStartMs.store(resumeMs);
    self->playback.stopRequested.store(false);
    self->playback.paused.store(false);
    self->playback.pending.store(PlaybackRequest::PlayCurrent);
    lv_label_set_text(self->statusLabel, "Playing...");
}

// Unified play/pause: playing -> stop stream and pause; paused -> resume
// from captured position; stopped -> start (with persisted resume if
// tracking is enabled). Shared by the on-screen button and the headset
// Play/Pause media key (service::audio request latch, polled in pollTimerCb).
void handlePlayPauseToggle(Context* self) {
    if (self->playback.playing.load() && !self->playback.paused.load()) {
        pausePlayback(self);
    } else if (self->playback.paused.load() && !self->playback.playing.load()) {
        resumePausedPlayback(self);
    } else if (self->playback.paused.load()) {
        lv_label_set_text(self->statusLabel, "Pausing...");
    } else {
        startPlayback(self);
    }
}

void onPlayPauseCb(lv_event_t* e) {
    handlePlayPauseToggle(static_cast<Context*>(lv_event_get_user_data(e)));
}

void trackNext(Context* self) {
    maybeSaveSidecarNow(self);
    self->playback.paused.store(false);
    if (self->playback.playing.load()) {
        // Cursor advances on the playback thread; the poll timer reloads
        // the sidecar once the now-playing path actually changes.
        self->playback.skipNext.store(true);
    } else {
        {
            std::lock_guard lg(self->playback.playlistMutex);
            int idx = self->playback.playlistIndex.load();
            int size = static_cast<int>(self->playback.playlist.size());
            if (idx + 1 < size) {
                self->playback.playlistIndex.store(idx + 1);
            }
        }
        loadSidecarForCurrentTrack(self);
        self->playback.pending.store(PlaybackRequest::PlayCurrent);
    }
}

void trackPrev(Context* self) {
    maybeSaveSidecarNow(self);
    self->playback.paused.store(false);
    if (self->playback.playing.load()) {
        self->playback.skipPrev.store(true);
    } else {
        {
            std::lock_guard lg(self->playback.playlistMutex);
            int idx = self->playback.playlistIndex.load();
            if (idx > 0) {
                self->playback.playlistIndex.store(idx - 1);
            }
        }
        loadSidecarForCurrentTrack(self);
        self->playback.pending.store(PlaybackRequest::PlayCurrent);
    }
}

void chapterPrev(Context* self) {
    int64_t pos = self->playback.currentPositionMs.load();
    int64_t target = 0;
    {
        std::lock_guard lg(self->sidecarMutex);
        // 2 s grace so a quick double-tap goes back further, not to "now".
        int64_t threshold = pos - 2000;
        for (const auto& c : self->currentSidecar.chapters) {
            if (c.start_ms <= threshold) target = c.start_ms;
            else break;
        }
    }
    requestSeekMs(self, target);
}

void chapterNext(Context* self) {
    int64_t pos = self->playback.currentPositionMs.load();
    int64_t target = -1;
    {
        std::lock_guard lg(self->sidecarMutex);
        for (const auto& c : self->currentSidecar.chapters) {
            if (c.start_ms > pos + 500) { target = c.start_ms; break; }
        }
    }
    if (target >= 0) requestSeekMs(self, target);
}

// Edge buttons are mode-aware: chapter jumps for audiobooks, track
// prev/next for music (audiobook mode intentionally has no next-book).
void onEdgeLeftCb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    bool audiobookMode;
    {
        std::lock_guard lg(self->sidecarMutex);
        audiobookMode = (self->currentSidecar.kind == "audiobook");
    }
    if (audiobookMode) chapterPrev(self);
    else trackPrev(self);
}

void onEdgeRightCb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    bool audiobookMode;
    {
        std::lock_guard lg(self->sidecarMutex);
        audiobookMode = (self->currentSidecar.kind == "audiobook");
    }
    if (audiobookMode) chapterNext(self);
    else trackNext(self);
}

void onAutoplaySwitchCb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    auto* sw = lv_event_get_target_obj(e);
    bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    self->playback.autoplay.store(enabled);
}

// Enable/disable resume tracking for the current track. Enabling writes the
// tracking file immediately so the feature works even if the app exits
// before the first autosave; disabling keeps the file (chapters survive)
// but stops position updates and resume priming.
void setTrackingEnabled(Context* self, bool enabled) {
    std::string sidecarPath;
    SidecarData snapshot;
    {
        std::lock_guard lg(self->sidecarMutex);
        if (self->currentSidecarPath.empty()) return;
        self->currentSidecar.tracking_enabled = enabled;
        if (enabled) {
            int64_t pos = self->playback.currentPositionMs.load();
            int64_t bytes = self->playback.currentFileBytes.load();
            if (pos > 0) {
                self->currentSidecar.last_position_ms = pos;
                self->currentSidecar.last_position_bytes = bytes;
            }
            snapshot = self->currentSidecar;
            sidecarPath = sidecarPathFor(self->currentSidecarPath, self->sidecarWorker.dir);
        }
    }
    if (enabled && !sidecarPath.empty()) {
        self->sidecarWorker.requestSave(sidecarPath, snapshot);
    }
    lv_label_set_text(self->statusLabel, enabled ? "Resume tracking on." : "Resume tracking off.");
}

void onTrackingSwitchCb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    auto* sw = lv_event_get_target_obj(e);
    setTrackingEnabled(self, lv_obj_has_state(sw, LV_STATE_CHECKED));
}

// The SliderBox owns its own slider/+/-/value-label; the event target is the
// box that changed, and the user data is the app Context.
void onVolumeChangedCb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    self->lastVolumeUserMs = lv_tick_get();
    auto* sliderBox = static_cast<lv_obj_t*>(lv_event_get_target(e));
    service::audio::setOutputVolume(static_cast<float>(lvgl_sliderbox_get_value(sliderBox)));
    if (service::audio::isOutputMuted()) {
        service::audio::setOutputMuted(false);
    }
}

// Enqueue a folder scan. The worker does the SD listing; the poll timer
// installs the result via applyReadyPlaylistIfAny().
void adoptPlaylistFromFolder(Context* self, const std::string& pickedPath) {
    LOG_I(TAG, "adoptPlaylistFromFolder enqueue picked='%s'", pickedPath.c_str());
    self->sidecarWorker.requestAdoptPlaylist(pickedPath);
}

void applyReadyPlaylistIfAny(Context* self) {
    if (!self->sidecarWorker.playlistReady.load()) return;
    std::vector<std::string> files;
    int idx = -1;
    std::string picked;
    {
        std::lock_guard lg(self->sidecarWorker.playlistReadyMutex);
        files = std::move(self->sidecarWorker.playlistReadyFiles);
        idx = self->sidecarWorker.playlistReadyIndex;
        picked = std::move(self->sidecarWorker.playlistReadyPicked);
        self->sidecarWorker.playlistReadyIndex = -1;
    }
    self->sidecarWorker.playlistReady.store(false);
    if (files.empty()) {
        LOG_W(TAG, "No MP3 files found for %s", picked.c_str());
        return;
    }
    self->libraryRootPath = dirnameOf(picked);
    {
        std::lock_guard lg(self->playback.playlistMutex);
        self->playback.playlist = std::move(files);
        self->playback.playlistIndex.store(idx);
    }
    loadSidecarForCurrentTrack(self);
    publishNowPlaying(&self->playback, picked);
    LOG_I(TAG, "Playlist loaded: %d entries, starting at %d",
          (int) self->playback.playlist.size(), idx);
}

void applyReadyScanIfAny(Context* self) {
    if (!self->sidecarWorker.scanReady.load()) return;
    std::string root;
    size_t count = 0;
    bool saved = false;
    {
        std::lock_guard lg(self->sidecarWorker.scanMutex);
        root = self->sidecarWorker.scanRoot;
        count = self->sidecarWorker.scanCount;
        saved = self->sidecarWorker.scanSaved;
    }
    self->sidecarWorker.scanReady.store(false);
    char buf[96];
    snprintf(buf, sizeof(buf),
             saved ? "Indexed %zu tracks." : "Index failed (%zu tracks).",
             count);
    if (self->statusLabel != nullptr) lv_label_set_text(self->statusLabel, buf);
    LOG_I(TAG, "Library scan root='%s' count=%zu saved=%d",
          root.c_str(), count, (int) saved);
}

// Request a sidecar load for an explicit MP3 path. The worker performs the
// read; the poll timer adopts the result via adoptLoadedSidecarIfReady().
void loadSidecarForPath(Context* self, const std::string& path) {
    {
        std::lock_guard lg(self->sidecarMutex);
        if (path == self->sidecarLoadPendingPath) return; // already in flight
        self->sidecarLoadPendingPath = path;
    }
    self->sidecarWorker.requestLoad(path);
}

// Load the sidecar for the currently-selected track (playlist[idx]).
void loadSidecarForCurrentTrack(Context* self) {
    std::string path;
    {
        std::lock_guard lg(self->playback.playlistMutex);
        int idx = self->playback.playlistIndex.load();
        if (idx < 0 || idx >= (int) self->playback.playlist.size()) return;
        path = self->playback.playlist[idx];
    }
    loadSidecarForPath(self, path);
}

// Current location: <appDataDir>/lastplayed.txt (SD card per platform
// convention). Older builds used the AppPaths (/data) location; migrate.
std::string lastPlayedStatePath(Context* self) {
    return self->appDataDir + "/" + LAST_PLAYED_FILE;
}

std::string legacyLastPlayedStatePath(Context* /*self*/) {
    // AppPaths-based location from before the SD convention + app id rename.
    return std::string("/data/tactility/user/app/") + LEGACY_APP_ID + "/" + LAST_PLAYED_FILE;
}

void saveLastPlayed(Context* self, const std::string& path) {
    if (path == self->savedLastPath) return;
    self->savedLastPath = path;
    self->sidecarWorker.requestSaveLastPlayed(path);
}

void applyReadyLastPlayedIfAny(Context* self) {
    if (!self->sidecarWorker.lastPlayedReady.load()) return;
    std::string path;
    {
        std::lock_guard lg(self->sidecarWorker.lastPlayedMutex);
        path = self->sidecarWorker.lastPlayedReadyPath;
        self->sidecarWorker.lastPlayedReadyPath.clear();
    }
    self->sidecarWorker.lastPlayedReady.store(false);
    if (!path.empty()) {
        self->lastPlayedFilePath = path;
        self->savedLastPath = path;
        adoptPlaylistFromFolder(self, path);
    }
}

// Old onCreate(): resolve per-app user-data locations and seed the worker's
// paths. Called from appMain() before the window is created.
void initAppData(Context* self) {
    self->playback.shutdown.store(false);
    self->playback.pending.store(PlaybackRequest::None);

    // App data follows the platform convention: with user data on the SD
    // card (CONFIG_TT_USER_DATA_LOCATION_SD), tt::getAppUserPath resolves
    // to /sdcard/tactility/user/app/one.tactility.audio -- the same per-app
    // organization other Tactility apps use. Media folders stay clean.
    self->appDataDir = tt::getAppUserPath(APP_ID);
    // One-time migration from the pre-rename app id: move the whole tree.
    std::string oldAppDir = tt::getAppUserPath(LEGACY_APP_ID);
    self->sidecarWorker.appDataDir = self->appDataDir;
    self->sidecarWorker.oldAppDir = oldAppDir;
    self->sidecarWorker.dir = self->appDataDir + "/" + RESUME_TRACKING_DIR;
    self->sidecarWorker.lastPlayedPath = lastPlayedStatePath(self);
    self->sidecarWorker.legacyLastPlayedPath = legacyLastPlayedStatePath(self);
    self->sidecarWorker.libraryIndexPath = self->appDataDir + "/" + LIBRARY_INDEX_FILE;
    self->sidecarWorker.legacyDirs = {
        // Interim internal-/data location (AppPaths, pre-rename app id).
        std::string("/data/tactility/user/app/") + LEGACY_APP_ID + "/sidecars",
        // Previous subdir name under the (migrated) app dir.
        self->appDataDir + "/sidecars",
    };
}

// Called from createWidgets rather than initAppData so the widget tree exists
// before the worker's first job starts hitting the SD card. Idempotent:
// createWidgets runs again after every return from the file picker.
void ensureBackendStarted(Context* self) {
    if (self->backendStarted) {
        return;
    }
    self->backendStarted = true;

    self->sidecarWorker.start();
    self->sidecarWorker.requestInitStorage();

    BaseType_t ok = xTaskCreate(
        &playbackTaskEntry, "audio_pb", 12 * 1024,
        &self->playback, tskIDLE_PRIORITY + 3, &self->playback.task);
    if (ok != pdPASS) {
        LOG_E(TAG, "Failed to create playback task");
        self->playback.task = nullptr;
    }
}

// Small helper so the transport/editor rows stay declarative below. Returns
// the button; `outLabel` receives its centered label when the caller needs
// to retarget the text later (play/pause symbol, mode-dependent edges).
lv_obj_t* addButton(Context* self, lv_obj_t* parent, const char* text, lv_event_cb_t callback,
                    lv_obj_t** outLabel = nullptr) {
    lv_obj_t* button = lv_btn_create(parent);
    lv_obj_set_height(button, LV_SIZE_CONTENT);
    lv_obj_set_flex_grow(button, 1);
    lv_obj_set_style_pad_ver(button, 9, LV_PART_MAIN);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, self);
    lv_obj_t* label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    if (outLabel != nullptr) {
        *outLabel = label;
    }
    return button;
}

// A borderless, zero-padding flex row - the building block for every
// control row, so rows don't inherit lv_obj's default card styling.
lv_obj_t* addRow(lv_obj_t* parent, int32_t gap) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row, gap, LV_PART_MAIN);
    lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, LV_PART_MAIN);
    return row;
}

// Label + switch pair for the toggle row.
lv_obj_t* addToggle(Context* self, lv_obj_t* parent, const char* text, bool checked, lv_event_cb_t callback) {
    lv_obj_t* column = lv_obj_create(parent);
    lv_obj_remove_flag(column, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_height(column, LV_SIZE_CONTENT);
    lv_obj_set_flex_grow(column, 1);
    lv_obj_set_flex_flow(column, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(column, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(column, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(column, 2, LV_PART_MAIN);
    lv_obj_set_style_border_width(column, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(column, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_t* label = lv_label_create(column);
    lv_label_set_text(label, text);

    lv_obj_t* toggle = lv_switch_create(column);
    if (checked) {
        lv_obj_add_state(toggle, LV_STATE_CHECKED);
    }
    lv_obj_add_event_cb(toggle, callback, LV_EVENT_VALUE_CHANGED, self);
    return toggle;
}

void onToolbarCloseCb(lv_event_t* e) {
    auto* self = static_cast<Context*>(lv_event_get_user_data(e));
    app_event_emit_close(self->appInstanceId);
}

// Old onShow(): build the widget tree. Runs on the LVGL thread, once when the
// window is created and again whenever this window resurfaces as topmost
// (e.g. after the file picker closes).
void createWidgets(lv_obj_t* parent, void* userData) {
    auto* self = static_cast<Context*>(userData);

    lv_obj_remove_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(parent, 0, LV_STATE_DEFAULT);

    lv_obj_t* toolbar = lvgl_toolbar_create(parent, "Audio");
    lvgl_toolbar_set_nav_action(toolbar, LV_SYMBOL_CLOSE, &onToolbarCloseCb, self);

    lv_obj_t* body = lv_obj_create(parent);
    lv_obj_remove_flag(body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(body, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(body, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(body, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_row(body, 7, LV_PART_MAIN);
    lv_obj_set_style_border_width(body, 0, LV_PART_MAIN);
    lv_obj_set_flex_grow(body, 1);
    lv_obj_set_width(body, LV_PCT(100));

    // Title. Dots (not circular scroll) on purpose: a marquee re-invalidates
    // the panel continuously, which this SPI display pays for every frame.
    self->nowPlayingLabel = lv_label_create(body);
    lv_label_set_long_mode(self->nowPlayingLabel, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_set_width(self->nowPlayingLabel, LV_PCT(100));
    lv_obj_set_style_text_align(self->nowPlayingLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(self->nowPlayingLabel, "(nothing playing)");

    self->infoLabel = lv_label_create(body);
    lv_label_set_long_mode(self->infoLabel, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_set_width(self->infoLabel, LV_PCT(100));
    lv_obj_set_style_text_align(self->infoLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(self->infoLabel, "No track selected");

    self->progressBar = lv_bar_create(body);
    lv_obj_set_width(self->progressBar, LV_PCT(100));
    lv_obj_set_height(self->progressBar, 8);
    lv_bar_set_range(self->progressBar, 0, 1000);
    lv_bar_set_value(self->progressBar, 0, LV_ANIM_OFF);

    self->progressLabel = lv_label_create(body);
    lv_obj_set_style_text_align(self->progressLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(self->progressLabel, "0:00");

    // Transport. The edge buttons are chapter jumps in audiobook mode and
    // track skips in music mode; refreshFromPlaybackState() retargets their
    // labels, so they are created with the music-mode symbols.
    lv_obj_t* transportRow = addRow(body, 5);
    addButton(self, transportRow, LV_SYMBOL_PREV, &onEdgeLeftCb, &self->edgeLeftLabel);
    addButton(self, transportRow, "-30", &onBack30Cb);
    addButton(self, transportRow, LV_SYMBOL_PLAY, &onPlayPauseCb, &self->playPauseLabel);
    addButton(self, transportRow, "+30", &onFwd30Cb);
    addButton(self, transportRow, LV_SYMBOL_NEXT, &onEdgeRightCb, &self->edgeRightLabel);

    lv_obj_t* volumeRow = addRow(body, 6);
    lv_obj_t* volumeCaption = lv_label_create(volumeRow);
    lv_label_set_text(volumeCaption, LV_SYMBOL_VOLUME_MAX);
    lv_obj_t* volumeSliderBox = lvgl_sliderbox_create(
        volumeRow, 0, 100, 5, static_cast<int32_t>(service::audio::getOutputVolume()));
    lv_obj_set_flex_grow(volumeSliderBox, 1);
    self->volumeSliderBox = volumeSliderBox;
    lvgl_sliderbox_add_value_changed_cb(volumeSliderBox, &onVolumeChangedCb, self);

    bool trackingOn = false;
    {
        std::lock_guard lg(self->sidecarMutex);
        trackingOn = self->currentSidecar.tracking_enabled;
    }
    lv_obj_t* toggleRow = addRow(body, 4);
    addToggle(self, toggleRow, "Autoplay", self->playback.autoplay.load(), &onAutoplaySwitchCb);
    self->trackingSwitch = addToggle(self, toggleRow, "Resume", trackingOn, &onTrackingSwitchCb);
    addToggle(self, toggleRow, "Edit", false, &onEditModeSwitchCb);

    // Chapter editor, revealed by the Edit toggle.
    self->editorRow = addRow(body, 4);
    lv_obj_add_flag(self->editorRow, LV_OBJ_FLAG_HIDDEN);
    addButton(self, self->editorRow, "Mark", &onMarkChapterCb);
    addButton(self, self->editorRow, "-5s", &onNudgeBack5Cb);
    addButton(self, self->editorRow, "-1s", &onNudgeBack1Cb);
    addButton(self, self->editorRow, "+1s", &onNudgeFwd1Cb);
    addButton(self, self->editorRow, "+5s", &onNudgeFwd5Cb);

    lv_obj_t* libraryRow = addRow(body, 6);
    addButton(self, libraryRow, LV_SYMBOL_DIRECTORY " Pick MP3", &onPickFileCb);
    addButton(self, libraryRow, LV_SYMBOL_REFRESH " Rescan", &onRescanCb);

    self->statusLabel = lv_label_create(body);
    lv_label_set_long_mode(self->statusLabel, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_set_width(self->statusLabel, LV_PCT(100));
    lv_obj_set_style_text_align(self->statusLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(self->statusLabel, "Pick an MP3 to begin.");

    // The refresh path is change-driven (dirty flag + throttles), so on a
    // re-show - returning from the file picker, for instance - it would skip
    // widgets that are already correct in state but brand new on screen.
    // Force the next refresh to repaint everything.
    self->playback.nowPlayingDirty.store(true);
    self->lastRenderedPositionSec = -1;
    self->lastPlayPauseWasPause = !(self->playback.playing.load() && !self->playback.paused.load());

    refreshFromPlaybackState(self);
    // A Play/Pause request made while the app was closed must not fire on open.
    service::audio::clearPlayPauseRequest();
    self->pollTimer = lv_timer_create(&pollTimerCb, 500, self);

    // Starts the sidecar worker and playback task on first show. This also
    // kicks off the last-played restore, so Play works right after a reboot
    // without picking a file again.
    ensureBackendStarted(self);
}

// Old onHide(): the widgets are about to be deleted while the app keeps
// running (e.g. the file picker window was created on top). Save progress,
// drop the poll timer and forget the widget pointers. Runs on the LVGL
// thread with the LVGL lock held; touches nothing that needs another lock
// held elsewhere (sidecarMutex is only ever taken by LVGL-thread code).
void destroyWidgets(void* userData) {
    auto* self = static_cast<Context*>(userData);

    maybeSaveSidecarNow(self);
    if (self->pollTimer != nullptr) {
        lv_timer_del(self->pollTimer);
        self->pollTimer = nullptr;
    }
    self->nowPlayingLabel = nullptr;
    self->infoLabel = nullptr;
    self->progressBar = nullptr;
    self->progressLabel = nullptr;
    self->playPauseLabel = nullptr;
    self->edgeLeftLabel = nullptr;
    self->edgeRightLabel = nullptr;
    self->statusLabel = nullptr;
    self->trackingSwitch = nullptr;
    self->editorRow = nullptr;
    self->volumeSliderBox = nullptr;
}

// Old onResult(): the FileSelection child app reports back through the
// AppStream bound to it; the picked path is its stdout payload.
void handleFileSelectionResult(Context* self, int32_t result) {
    if (result == 0 /* Ok */) {
        char buffer[sizeof(self->fileResultBuffer)];
        size_t length = app_stream_read(&self->fileResultStream, buffer, sizeof(buffer));
        std::string picked(buffer, length);
        LOG_I(TAG, "FileSelection returned path='%s' isMp3=%d",
              picked.c_str(), (int) hasMp3Extension(picked));
        if (!picked.empty() && hasMp3Extension(picked)) {
            self->lastPlayedFilePath = picked;
            self->libraryRootPath = dirnameOf(picked);
            adoptPlaylistFromFolder(self, picked);
            saveLastPlayed(self, picked);
            LOG_I(TAG, "Selected: %s", picked.c_str());
        }
    }
    app_stream_unsubscribe(&self->fileResultStream);
}

int32_t appMain(int argc, char* argv[]) {
    (void) argc;
    (void) argv;

    uint32_t appInstanceId = app_scheduler_current_app_id();

    Context self {};
    self.appInstanceId = appInstanceId;

    TaskEventGroup eventGroup {};
    task_event_group_construct(&eventGroup);
    self.eventGroup = &eventGroup;

    // Old onCreate().
    initAppData(&self);

    AppEventSubscription sub {};
    check(app_event_subscribe(&sub, &eventGroup) == ERROR_NONE);

    WindowId window = window_manager_create_ext(appInstanceId, createWidgets, destroyWidgets, &self);

    bool shouldClose = false;
    while (!shouldClose) {
        task_event_group_wait_any(&eventGroup, nullptr, portMAX_DELAY);

        AppEvent event {};
        while (app_event_poll(&sub, &event) == ERROR_NONE) {
            switch (event.type) {
                case APP_EVENT_CLOSE:
                    shouldClose = true;
                    break;
                case APP_EVENT_RESULT:
                    // Old onResult().
                    if (event.result.launch_id == self.fileSelectionLaunchId) {
                        self.fileSelectionLaunchId = 0;
                        handleFileSelectionResult(&self, event.result.result);
                    }
                    app_manager_stop(event.result.launch_id);
                    break;
                default:
                    break;
            }
            if (shouldClose) break;
        }
    }

    // Old onHide() + onDestroy(): flush the resume position, tear down the
    // playback task and the sidecar worker (its queue is FIFO, so a pending
    // position save drains before the Shutdown job), then the window itself.
    maybeSaveSidecarNow(&self);
    if (lvgl_try_lock(1000 / portTICK_PERIOD_MS)) {
        if (self.pollTimer != nullptr) {
            lv_timer_del(self.pollTimer);
            self.pollTimer = nullptr;
        }
        lvgl_unlock();
    }
    self.playback.stopRequested.store(true);
    self.playback.shutdown.store(true);
    for (int i = 0; i < 80 && self.playback.task != nullptr; ++i) {
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    // Queue is FIFO: the Shutdown job lands after any pending save, so the
    // final position write completes before the worker exits.
    self.sidecarWorker.stop();

    window_manager_remove(window);
    check(app_event_unsubscribe(&sub) == ERROR_NONE);
    task_event_group_destruct(&eventGroup);

    return 0;
}

} // namespace

extern const ::AppManifest manifest = {
    .id = "tactility.audio",
    .name = "Audio Player",
    .category = APP_CATEGORY_USER,
    .location = { APP_LOCATION_MEMORY, reinterpret_cast<void*>(appMain) },
};

} // namespace tt::app::audio
