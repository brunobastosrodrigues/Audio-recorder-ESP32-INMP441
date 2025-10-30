/*
  ESP32 + INMP441-like I2S mic
  -> BLE GATT (IMA-ADPCM) + WAV recorder (SPIFFS) + HTTP file browser
  + Push-to-record (hold BOOT), silence-watchdog, gain & Wi‑Fi hotkeys

  Quick controls (Serial, 115200):
    1 = LEFT/STD     2 = RIGHT/STD     3 = LEFT/MSB     4 = RIGHT/MSB
    + = louder (lower SHIFT)           - = quieter (raise SHIFT)
    r = start/stop recording (manual)  h = help
    w = toggle Wi‑Fi AP on/off         W = force AP ON     X = force AP OFF

  HTTP file browser:
    AP SSID: ESP32-Recorder     PASS: micglass
    URL: http://192.168.4.1/

  Mic wiring:
    VDD      -> 3V3
    GND      -> GND
    BCLK/SCK -> ESP32 GPIO26
    LRCK/WS  -> ESP32 GPIO25
    DOUT/SD  -> ESP32 GPIO32
    L/R (SEL)-> 3V3  (RIGHT slot; use hotkeys if your board differs)

  Before flashing: Tools → Partition Scheme → No OTA (2MB APP/2MB SPIFFS) or any with ≥1MB SPIFFS
*/

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#include "driver/i2s.h"
#include <NimBLEDevice.h>

// ---------- FS (SPIFFS) ----------
#include <FS.h>
#include "SPIFFS.h"

// ---------- Wi‑Fi + HTTP ----------
#include <WiFi.h>
#include <WebServer.h>
WebServer web(80);
static const char* AP_SSID = "ESP32-Recorder";
static const char* AP_PASS = "micglass"; // change if desired

// ---------- FreeRTOS ticks helper ----------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifndef pdMS_TO_TICKS
  #define pdMS_TO_TICKS(ms) ((ms) / portTICK_PERIOD_MS)
#endif

// -------------------- USER CONFIG --------------------
#define BLE_NAME              "ESP32-ADPCM-Mic"
#define SAMPLE_RATE           16000   // Hz
#define CAPTURE_SAMPLES       1024    // samples per block (mono)
#define SHIFT_BITS_DEFAULT    11      // try 10..13; lower = louder
#define DEBUG_RAW_SAMPLES     1       // print first 8 raw I2S words ~1/s
#define DEBUG_AUDIO_STATS     1       // RMS/Peak ~1/s

// (Set to 0 to keep AP always on)
#define PAUSE_WIFI_DURING_REC 1

// (Optional) Simple high-pass filter (~120 Hz) to remove rumble/DC
#define HPF_ENABLE            1
#define HPF_FC_HZ             120

// Silence watchdog: rotate I2S mode if blocks look "all same" for a while
#define SILENCE_WATCHDOG            1
#define SILENCE_BLOCKS_THRESHOLD    32   // ~2 s worth of blocks

// I2S pins
#define I2S_BCLK_GPIO 26
#define I2S_LRCK_GPIO 25
#define I2S_DATA_GPIO 32

// BOOT button (active LOW)
#define REC_BTN_PIN   0

// ---- I2S mode constants (ints avoid Arduino auto-prototype issues)
#define CH_LEFT   0
#define CH_RIGHT  1
#define FMT_STD   0
#define FMT_MSB   1

// ---- Forward decl
static bool install_i2s(int ch, int fmt);

// BLE UUIDs (random)
static const char* SERVICE_UUID    = "7d8fb7b0-2aa2-4b0f-b8ef-0f8f7e9f0c11";
static const char* CHAR_AUDIO_UUID = "9a1e0fc5-6a63-4f03-8d7d-9b1d7a9be0a2";

// ---------- BLE state ----------
static NimBLEServer*         g_server = nullptr;
static NimBLECharacteristic* g_charAudio = nullptr;
static volatile bool         g_clientConnected = false;
static uint8_t               g_seq = 0;

// ---------- Audio / ADPCM ----------
static int g_shiftBits = SHIFT_BITS_DEFAULT;
struct IMAState { int16_t predictor = 0; int index = 0; };

static const int stepTable[89] = {
   7, 8, 9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,
  50,55,60,66,73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,
  307,337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,
  1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,
  7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,
  24623,27086,29794,32767
};
static const int indexTable[16] = {
  -1,-1,-1,-1, 2,4,6,8, -1,-1,-1,-1, 2,4,6,8
};

static inline uint8_t adpcm_encode_sample(int16_t sample, IMAState& st) {
  int step = stepTable[st.index];
  int diff = sample - st.predictor;
  uint8_t code = 0;
  if (diff < 0) { code = 8; diff = -diff; }
  int tmp = step;
  if (diff >= tmp) { code |= 4; diff -= tmp; }
  tmp >>= 1; if (diff >= tmp) { code |= 2; diff -= tmp; }
  tmp >>= 1; if (diff >= tmp) { code |= 1; }

  int delta = (step >> 3);
  if (code & 4) delta += step;
  if (code & 2) delta += step >> 1;
  if (code & 1) delta += step >> 2;

  if (code & 8) st.predictor -= delta; else st.predictor += delta;
  if (st.predictor > 32767) st.predictor = 32767;
  if (st.predictor < -32768) st.predictor = -32768;

  st.index += indexTable[code & 0x0F];
  if (st.index < 0) st.index = 0;
  if (st.index > 88) st.index = 88;
  return code & 0x0F;
}
static void ima_adpcm_encode_block(const int16_t* pcm, int n, uint8_t* out, IMAState& st) {
  int outPos = 0;
  for (int i = 0; i < n; i += 2) {
    uint8_t n0 = adpcm_encode_sample(pcm[i], st);
    uint8_t n1 = (i+1 < n) ? adpcm_encode_sample(pcm[i+1], st) : n0;
    out[outPos++] = (n1 << 4) | n0;
  }
}

// ---------- Debug helpers ----------
static float rms_dbfs(const int16_t* x, int n) {
  long long acc = 0;
  for (int i = 0; i < n; ++i) { long v = x[i]; acc += (long long)v*v; }
  float rms = sqrtf((float)acc / (float)n) / 32768.0f;
  if (rms <= 0) return -120.0f;
  return 20.0f * log10f(rms);
}
static float peak_dbfs(const int16_t* x, int n) {
  int16_t peak = 0;
  for (int i = 0; i < n; ++i) {
    int16_t a = x[i] < 0 ? (int16_t)(-x[i]) : x[i];
    if (a > peak) peak = a;
  }
  if (peak == 0) return -120.0f;
  return 20.0f * log10f((float)peak / 32768.0f);
}

// ---------- BLE callbacks ----------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s) {
    g_clientConnected = true;
    Serial.println("[BLE] Client connected.");
  }
  void onDisconnect(NimBLEServer* s) {
    g_clientConnected = false;
    Serial.println("[BLE] Client disconnected. Restarting advertising.");
    s->startAdvertising();
  }
};

// ---------- I2S install ----------
static bool g_i2sInstalled = false;
static int  g_curChan = CH_RIGHT; // default to RIGHT/MSB (works on many boards)
static int  g_curFmt  = FMT_MSB;

static void uninstall_i2s() {
  if (g_i2sInstalled) { i2s_driver_uninstall(I2S_NUM_0); g_i2sInstalled = false; }
}
static bool install_i2s(int ch, int fmt) {
  uninstall_i2s();

  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = (ch == CH_RIGHT) ? I2S_CHANNEL_FMT_ONLY_RIGHT
                                       : I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (fmt == FMT_STD)
      ? I2S_COMM_FORMAT_STAND_I2S
      : (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = 256,
    .use_apll = true,        // slightly cleaner clock
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num   = I2S_BCLK_GPIO,
    .ws_io_num    = I2S_LRCK_GPIO,
    .data_out_num = -1,
    .data_in_num  = I2S_DATA_GPIO
  };

  esp_err_t e = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  if (e != ESP_OK) { Serial.printf("[I2S] install error: %d\n", e); return false; }
  e = i2s_set_pin(I2S_NUM_0, &pins);
  if (e != ESP_OK) { Serial.printf("[I2S] set_pin error: %d\n", e); return false; }
  e = i2s_zero_dma_buffer(I2S_NUM_0);
  if (e != ESP_OK) { Serial.printf("[I2S] zero_dma error: %d\n", e); }

  g_i2sInstalled = true; g_curChan = ch; g_curFmt = fmt;
  Serial.printf("[I2S] Installed: SR=%d, Chan=%s, Fmt=%s, Pins BCLK=%d WS=%d DIN=%d, SHIFT=%d\n",
                SAMPLE_RATE, (ch==CH_RIGHT?"RIGHT":"LEFT"),
                (fmt==FMT_STD ? "STAND_I2S" : "I2S|MSB"),
                I2S_BCLK_GPIO, I2S_LRCK_GPIO, I2S_DATA_GPIO, g_shiftBits);
  return true;
}

// ---------- Silence watchdog + mode rotation ----------
struct Mode { int ch; int fmt; const char* label; };
static Mode kModes[4] = {
  {CH_LEFT,  FMT_STD, "LEFT/STAND_I2S"},
  {CH_RIGHT, FMT_STD, "RIGHT/STAND_I2S"},
  {CH_LEFT,  FMT_MSB, "LEFT/I2S|MSB"},
  {CH_RIGHT, FMT_MSB, "RIGHT/I2S|MSB"}
};
static int  g_modeIndex = 3; // start at RIGHT/MSB
static int  g_silenceBlocks = 0;

static bool block_all_same(const int32_t* buf, int n, uint32_t* sameValOut) {
  int32_t mn = INT32_MAX, mx = INT32_MIN;
  for (int i = 0; i < n; ++i) { if (buf[i] < mn) mn = buf[i]; if (buf[i] > mx) mx = buf[i]; }
  if (mn == mx) { if (sameValOut) *sameValOut = (uint32_t)mn; return true; }
  return false;
}
static void rotate_mode() {
  g_modeIndex = (g_modeIndex + 1) % 4;
  install_i2s(kModes[g_modeIndex].ch, kModes[g_modeIndex].fmt);
  Serial.printf("[I2S] Silence watchdog: switching to %s\n", kModes[g_modeIndex].label);
  g_silenceBlocks = 0;
}

// ---------- (Optional) High-pass filter ----------
#if HPF_ENABLE
struct HPFState { float y1 = 0.f; float x1 = 0.f; };
static HPFState g_hpf;
static inline void hpf_process(int16_t* pcm, int n) {
  const float rc = 1.0f / (2.0f * (float)M_PI * (float)HPF_FC_HZ);
  const float dt = 1.0f / (float)SAMPLE_RATE;
  const float a  = rc / (rc + dt);                       // y[n] = a*(y[n-1] + x[n] - x[n-1])
  for (int i = 0; i < n; ++i) {
    float x = (float)pcm[i] / 32768.0f;
    float y = a * (g_hpf.y1 + x - g_hpf.x1);
    g_hpf.y1 = y;
    g_hpf.x1 = x;
    if (y > 0.999969f) y = 0.999969f; else if (y < -1.0f) y = -1.0f;
    pcm[i] = (int16_t)lrintf(y * 32768.0f);
  }
}
#endif

// ---------- WAV writer (SPIFFS) with header keepalive ----------
class WavWriter {
public:
  bool begin(uint32_t sampleRate, uint16_t bitsPerSample = 16, uint16_t channels = 1) {
    if (!SPIFFS.begin(true)) { Serial.println("[FS] SPIFFS mount failed."); return false; }
    String path = nextFilename();
    SPIFFS.remove(path);
    f = SPIFFS.open(path, FILE_WRITE);
    if (!f) { Serial.println("[FS] Open for write failed."); return false; }
    _sr = sampleRate; _bps = bitsPerSample; _ch = channels; _dataBytes = 0;
    writeHeaderPlaceholder();
    Serial.printf("[FS] Recording to %s\n", path.c_str());
    return true;
  }
  bool writeSamples(const int16_t* s, size_t n) {
    if (!f) return false;
    size_t bytes = n * sizeof(int16_t);
    size_t w = f.write((const uint8_t*)s, bytes);
    _dataBytes += w;
    return w == bytes;
  }
  void patchHeader() {
    if (!f) return;
    size_t cur = f.position();
    uint32_t subchunk2 = _dataBytes;
    uint32_t riffSize  = 36 + subchunk2;
    f.seek(4, SeekSet);  writeLE32(riffSize);
    f.seek(40, SeekSet); writeLE32(subchunk2);
    f.seek(cur, SeekSet);
    f.flush();
  }
  bool finalize() {
    if (!f) return false;
    patchHeader();
    f.close();
    Serial.printf("[FS] Finalized WAV: data=%u bytes (%.2f s @ %u Hz)\n",
                  _dataBytes, (double)_dataBytes / (_ch * (_bps/8)) / _sr, _sr);
    return true;
  }
  bool active() const { return f; }
private:
  File f;
  uint32_t _sr = 0, _dataBytes = 0;
  uint16_t _bps = 16, _ch = 1;
  void writeLE16(uint16_t v) { uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) }; f.write(b,2); }
  void writeLE32(uint32_t v) { uint8_t b[4] = { (uint8_t)(v & 0xFF), (uint8_t)((v>>8)&0xFF), (uint8_t)((v>>16)&0xFF), (uint8_t)(v>>24) }; f.write(b,4); }
  void writeHeaderPlaceholder() {
    f.write((const uint8_t*)"RIFF", 4);
    writeLE32(0);
    f.write((const uint8_t*)"WAVE", 4);
    f.write((const uint8_t*)"fmt ", 4);
    writeLE32(16);  writeLE16(1); // PCM
    writeLE16(_ch); writeLE32(_sr);
    writeLE32(_sr * _ch * (_bps/8));
    writeLE16(_ch * (_bps/8));
    writeLE16(_bps);
    f.write((const uint8_t*)"data", 4);
    writeLE32(0);
  }
  String nextFilename() {
    for (int i = 0; i < 1000; ++i) {
      char tmp[20]; snprintf(tmp, sizeof(tmp), "/rec-%03d.wav", i);
      if (!SPIFFS.exists(tmp)) return String(tmp);
    }
    return String("/rec.wav");
  }
};

// ---------- Recorder state (single definitions) ----------
static WavWriter g_rec;
static bool      g_isRecording = false;
static bool      g_recByButton = false; // true if recording started by button press
static uint32_t  g_lastHeaderPatchMs = 0;

// ---------- HTTP helpers (robust path normalization) ----------
static bool g_webHandlersReady = false;

static String htmlHeader(const String& title) {
  String s = "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<title>" + title + "</title>";
  s += "<style>body{font-family:system-ui,Segoe UI,Arial;margin:20px}table{border-collapse:collapse}td,th{padding:8px 10px;border-bottom:1px solid #ddd}</style>";
  s += "</head><body><h2>" + title + "</h2>";
  return s;
}
static String humanSize(uint32_t b) {
  char buf[32];
  if (b >= 1024*1024) { snprintf(buf, sizeof(buf), "%.2f MB", b/1048576.0); }
  else if (b >= 1024) { snprintf(buf, sizeof(buf), "%.1f KB", b/1024.0); }
  else { snprintf(buf, sizeof(buf), "%u B", b); }
  return String(buf);
}
// Normalize & validate path: accept "rec-000.wav", "/rec-000.wav", "/spiffs/rec-000.wav"
static String normalizePath(String raw) {
  raw.trim();
  raw.replace("\\", "/");
  if (raw.startsWith("/spiffs/")) raw = raw.substring(7);
  if (raw.length() && raw[0] != '/') raw = "/" + raw;
  if (raw.indexOf("..") >= 0) return "";
  if (!raw.startsWith("/rec-") || !raw.endsWith(".wav")) return "";
  if (raw.substring(1).indexOf('/') >= 0) return ""; // only root
  return raw;
}
static void handleIndex() {
  if (!SPIFFS.begin(true)) { web.send(500, "text/plain", "SPIFFS mount failed"); return; }
  File root = SPIFFS.open("/");
  if (!root) { web.send(500, "text/plain", "Open / failed"); return; }
  String page = htmlHeader("ESP32 Recorder – Files");
  page += "<table><tr><th>Name</th><th>Size</th><th>Actions</th></tr>";
  File f = root.openNextFile();
  int count = 0;
  while (f) {
    String name = f.name(); if (name.length() && name[0] == '/') name.remove(0,1);
    uint32_t sz = f.size();
    page += "<tr><td>/" + name + "</td><td>" + humanSize(sz) + "</td>";
    page += "<td><a href='/download?f=" + name + "'>download</a> &nbsp; ";
    page += "<a href='#' onclick=\"if(confirm('Delete /" + name + "?'))location='/delete?f=" + name + "';return false;\">delete</a></td></tr>";
    f = root.openNextFile(); count++;
  }
  page += "</table>";
  if (count == 0) page += "<p><i>No files found. Hold BOOT to record or press 'r' in Serial.</i></p>";
  page += "</body></html>";
  web.send(200, "text/html; charset=utf-8", page);
}
static void handleDownload() {
  if (!web.hasArg("f")) { web.send(400, "text/plain", "Missing ?f=rec-xxx.wav"); return; }
  String canonical = normalizePath(web.arg("f"));
  if (canonical == "") { web.send(403, "text/plain", "Forbidden path"); return; }
  if (!SPIFFS.exists(canonical)) { web.send(404, "text/plain", "File not found"); return; }
  File f = SPIFFS.open(canonical, FILE_READ);
  if (!f) { web.send(500, "text/plain", "Open failed"); return; }
  String fname = canonical.substring(1);
  web.sendHeader("Content-Disposition", "attachment; filename=\"" + fname + "\"");
  web.streamFile(f, "audio/wav");
  f.close();
  Serial.printf("[HTTP] download %s OK\n", fname.c_str());
}
static void handleDelete() {
  if (!web.hasArg("f")) { web.send(400, "text/plain", "Missing ?f=rec-xxx.wav"); return; }
  String canonical = normalizePath(web.arg("f"));
  if (canonical == "") { web.send(403, "text/plain", "Forbidden path"); return; }
  if (!SPIFFS.exists(canonical)) { web.send(404, "text/plain", "File not found"); return; }
  SPIFFS.remove(canonical);
  web.sendHeader("Location", "/");
  web.send(303);
  Serial.printf("[HTTP] delete %s OK\n", canonical.substring(1).c_str());
}

static void registerHandlersOnce() {
  if (g_webHandlersReady) return;
  web.on("/", handleIndex);
  web.on("/download", handleDownload);
  web.on("/delete", handleDelete);
  web.onNotFound([]() {
    String msg = "Not Found: " + web.uri() + "\n";
    for (uint8_t i=0;i<web.args();++i) msg += "  " + web.argName(i) + " = " + web.arg(i) + "\n";
    web.send(404, "text/plain", msg);
  });
  g_webHandlersReady = true;
}

static bool g_wifiPaused = false;
static void startAP() {
  registerHandlersOnce();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  web.begin();
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[WIFI] AP \"%s\" up. Browse: http://%s/\n", AP_SSID, ip.toString().c_str());
}
static void stopAP() {
#if PAUSE_WIFI_DURING_REC
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  g_wifiPaused = true;
  Serial.println("[WIFI] AP paused.");
#endif
}
static void resumeAP() {
#if PAUSE_WIFI_DURING_REC
  registerHandlersOnce();          // idempotent
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  web.begin();
  IPAddress ip = WiFi.softAPIP();
  g_wifiPaused = false;
  Serial.printf("[WIFI] AP resumed. http://%s/\n", ip.toString().c_str());
#endif
}

// ---------- BLE setup ----------
static void setupBLE() {
  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(185);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());

  NimBLEService* svc = g_server->createService(SERVICE_UUID);
  g_charAudio = svc->createCharacteristic(CHAR_AUDIO_UUID,
                    NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  const char hdr[] = "IMA-ADPCM,16kHz,mono,block=1024";
  g_charAudio->setValue((uint8_t*)hdr, sizeof(hdr)-1);
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  NimBLEAdvertisementData advData;
  advData.setName(BLE_NAME);
  adv->setAdvertisementData(advData);
  adv->start();

  Serial.printf("[BLE] Advertising as \"%s\"  Service=%s\n", BLE_NAME, SERVICE_UUID);
}

// ---------- Helpers ----------
static void printHelp() {
  Serial.println("Hotkeys: 1=LEFT/STD  2=RIGHT/STD  3=LEFT/MSB  4=RIGHT/MSB  +=louder  -=quieter  r=start/stop  w=AP toggle  W=AP ON  X=AP OFF  h=help");
}

// Start/stop recording helpers
static void startRecording(bool byButton) {
  if (g_isRecording) return;
  stopAP(); // reduce RF noise if enabled
  if (g_rec.begin(SAMPLE_RATE, 16, 1)) {
    g_isRecording = true;
    g_recByButton = byButton;
#if HPF_ENABLE
    g_hpf = HPFState(); // reset HPF state
#endif
    g_lastHeaderPatchMs = millis();
    Serial.printf("[REC] Started (%s).\n", byButton ? "button" : "manual");
  } else {
    Serial.println("[REC] Failed to open file.");
  }
}
static void stopRecording() {
  if (!g_isRecording) return;
  g_rec.finalize();
  g_isRecording = false;
  g_recByButton = false;
  resumeAP(); // bring Wi‑Fi back if it was paused
  Serial.println("[REC] Stopped.");
}

// -------------------- Arduino --------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("[BOOT] ESP32 INMP441 -> BLE + WAV + HTTP (push-to-record + AP toggle)");
  Serial.printf("[BOOT] Free heap: %u bytes\n", (unsigned)ESP.getFreeHeap());
  pinMode(REC_BTN_PIN, INPUT_PULLUP);

  install_i2s(g_curChan, g_curFmt);         // start at RIGHT/MSB
  setupBLE();
  startAP();
  printHelp();

  Serial.println("[READY] Hold BOOT to record; release to finalize. Or press 'r' to start/stop.");
  Serial.println("[READY] Connect Wi‑Fi \"ESP32-Recorder\" (pass: micglass) → http://192.168.4.1/");
  Serial.println("[READY] In nRF Connect: connect → characteristic → Enable Notifications.");
}

void loop() {
  static int32_t i2sBuf[CAPTURE_SAMPLES];
  static int16_t pcm16[CAPTURE_SAMPLES];
  static uint8_t adpcm[CAPTURE_SAMPLES / 2];
  static IMAState encStream; // encoder state for the BLE stream
  static uint32_t blockId = 0;
  static uint32_t lastReportMs = 0;

  // Serve HTTP (if AP is up)
  web.handleClient();

  // --- Serial hotkeys ---
  if (Serial.available()) {
    int c = Serial.read();
    if (c=='1'||c=='2'||c=='3'||c=='4') {
      int idx = (c=='1')?0:(c=='2')?1:(c=='3')?2:3;
      g_modeIndex = idx;
      install_i2s(kModes[idx].ch, kModes[idx].fmt);
      Serial.printf("[I2S] Manual mode: %s\n", kModes[idx].label);
      g_silenceBlocks = 0;
    } else if (c=='+') {
      if (g_shiftBits>8) g_shiftBits--;
      Serial.printf("[GAIN] SHIFT=%d (lower is louder)\n", g_shiftBits);
    } else if (c=='-') {
      if (g_shiftBits<14) g_shiftBits++;
      Serial.printf("[GAIN] SHIFT=%d (lower is louder)\n", g_shiftBits);
    } else if (c=='r' || c=='R') {
      if (!g_isRecording) startRecording(false);
      else stopRecording();
    } else if (c=='w') {                    // toggle AP
      if (WiFi.getMode() == WIFI_OFF) resumeAP();
      else stopAP();
    } else if (c=='W') {                    // force AP ON
      resumeAP();
    } else if (c=='X') {                    // force AP OFF
      stopAP();
    } else if (c=='h' || c=='H') {
      printHelp();
    }
    while (Serial.available()) Serial.read(); // flush remainder
  }

  // --- Push-to-record on BOOT (active LOW) ---
  static bool btnPrev = true; // HIGH when released (pull-up)
  bool btn = digitalRead(REC_BTN_PIN);
  if (!btn && btnPrev) {                 // pressed
    startRecording(true);
  } else if (btn && !btnPrev) {          // released
    if (g_isRecording && g_recByButton) stopRecording();
  }
  btnPrev = btn;

  // --- I2S read ---
  size_t br = 0;
  esp_err_t e = i2s_read(I2S_NUM_0, (void*)i2sBuf, sizeof(i2sBuf), &br, portMAX_DELAY);
  if (e != ESP_OK) { Serial.printf("[I2S] read error: %d\n", e); delay(2); return; }
  int got = br / sizeof(int32_t);
  if (got <= 0) return;

  // Silence watchdog
  uint32_t sameVal = 0;
  bool allSame = block_all_same(i2sBuf, got, &sameVal);
#if SILENCE_WATCHDOG
  if (allSame) {
    g_silenceBlocks++;
    if (g_silenceBlocks == 4) {
      Serial.printf("[I2S] Input constant: 0x%08X (count=%d)\n", sameVal, g_silenceBlocks);
    }
    if (g_silenceBlocks > SILENCE_BLOCKS_THRESHOLD) {
      rotate_mode();
    }
  } else {
    if (g_silenceBlocks >= 4) Serial.println("[I2S] Signal detected; watchdog reset.");
    g_silenceBlocks = 0;
  }
#endif

#if DEBUG_RAW_SAMPLES
  static uint32_t lastRawMsLocal = 0;
  if (millis() - lastRawMsLocal > 1000) {
    Serial.print("[RAW] ");
    for (int i = 0; i < 8 && i < got; ++i) Serial.printf("%08X ", (uint32_t)i2sBuf[i]);
    Serial.println();
    lastRawMsLocal = millis();
  }
#endif

  // Convert to 16-bit PCM
  for (int i = 0; i < got; ++i) pcm16[i] = (int16_t)(i2sBuf[i] >> g_shiftBits);

#if HPF_ENABLE
  hpf_process(pcm16, got);
#endif

#if DEBUG_AUDIO_STATS
  if (millis() - lastReportMs > 1000) {
    float r = rms_dbfs(pcm16, got);
    float p = peak_dbfs(pcm16, got);
    Serial.printf("[AUDIO] block=%lu  RMS=%.1f dBFS  Peak=%.1f dBFS  cfg=%s/%s  shift=%d\n",
                  (unsigned long)blockId, r, p,
                  (g_curChan==CH_RIGHT?"RIGHT":"LEFT"),
                  (g_curFmt==FMT_STD?"STAND_I2S":"I2S|MSB"), g_shiftBits);
    if (r < -80.0f) Serial.println("[HINT] Very low level. Check BCLK/WS/DOUT and SEL (L/R). Use 1-4 to try other modes.");
    lastReportMs = millis();
  }
#endif

  // --- Write to WAV if recording ---
  if (g_isRecording) {
    g_rec.writeSamples(pcm16, got);
    // Patch header every 1 s so partial files stay valid
    uint32_t now = millis();
    if (now - g_lastHeaderPatchMs > 1000) {
      g_rec.patchHeader();
      g_lastHeaderPatchMs = now;
    }
  }

  // --- Encode to IMA-ADPCM + BLE notify ---
  ima_adpcm_encode_block(pcm16, got, adpcm, encStream);
  int adpcmBytes = got / 2; // 2 samples per byte

  if (g_clientConnected) {
    uint8_t header[4] = {0xAD, g_seq++, (uint8_t)(adpcmBytes & 0xFF), (uint8_t)(adpcmBytes >> 8)};
    g_charAudio->setValue(header, sizeof(header));
    g_charAudio->notify();

    const int MTU_PAYLOAD = 180;
    int remaining = adpcmBytes, offset = 0;
    while (remaining > 0) {
      int chunk = (remaining > MTU_PAYLOAD) ? MTU_PAYLOAD : remaining;
      g_charAudio->setValue(adpcm + offset, chunk);
      g_charAudio->notify();
      offset += chunk; remaining -= chunk;
      delay(1);
    }
  }

  ++blockId;
}
