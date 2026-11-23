#include <WiFi.h>
#include <FirebaseESP32.h>
#include <AccelStepper.h>
#include <Preferences.h>
#include "esp_task_wdt.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// =========================================================================
// DEFINISI ENUM & STRUCT DI PALING ATAS (MENCEGAH ERROR AUTO-PROTOTYPE ARDUINO IDE)
// =========================================================================
enum RunMode {
  MODE_IDLE,
  MODE_CONTINUOUS,
  MODE_POSITION,
  MODE_ASRS_SEQUENCE
};

enum AsrsStep {
  ASRS_NONE = 0,
  ASRS_STEP_3_MOVE_X = 1,          // Website stageCard-1
  ASRS_STEP_4_MOVE_Z = 2,          // Website stageCard-2
  ASRS_STEP_5A_SERVO_EXTEND = 3,   // Website stageCard-3
  ASRS_STEP_5B_Z_LIFT_DROP = 4,    // Website stageCard-4
  ASRS_STEP_5C_SERVO_RETRACT = 5,  // Website stageCard-5
  ASRS_STEP_6_BACK_HOME = 6,       // Website stageCard-6
  ASRS_STEP_5A_SETTLE = 7,
  ASRS_STEP_5C_SETTLE = 8
};

struct CommandPacket {
  char cmd[64];
  char source[12];
};

struct TelemetrySnapshot {
  char statusMsg[64];
  char box[8];
  char op[16];
  char servoState[12];
  bool locked;
  bool running;
  long m1Steps;
  long m2Steps;
  float m1Rev;
  float m2Rev;
  float servoExtRev;
  float servoRetRev;
  unsigned long servoMsRev;
  int stepStage;
  float rpm;
  float accel;
  float zOffsetRev;
  float boxX[3][3];
  float boxZ[3][3];
};

struct RestoredConfigPacket {
  bool valid;
  bool hasSteps;
  long m1Steps;
  long m2Steps;
  float rpm;
  float accel;
  float servoExtRev;
  float servoRetRev;
  unsigned long servoMsRev;
  float zOffsetRev;
  float boxX[3][3];
  float boxZ[3][3];
};

// Forward Declarations eksplisit untuk Arduino Preprocessor
int getUiStepStage(AsrsStep st);
void applyRestoredConfigOnCore1(const RestoredConfigPacket& cfg);
void saveConfigAndPositionToFirebase(const TelemetrySnapshot& snap);
void requestFirebaseSync(const char* statusMsg, bool saveConfigOnline = false);
void requestFirebaseSync(const String& statusMsg, bool saveConfigOnline = false);
void startAsrsCycle(const char* opType, const String& boxAddr, float customXRev = -1.0f, float customZRev = -1.0f);

// ==========================================
// KREDENSIAL WIFI & FIREBASE
// ==========================================
#define WIFI_SSID "moya"
#define WIFI_PASSWORD "PEMIMPINrakyat"

#define FIREBASE_HOST "pp-ind02-default-rtdb.asia-southeast1.firebasedatabase.app" 
#define FIREBASE_AUTH "nryzRKE5yCmDJqwdPtMOHDKV8EKbdTAraTgQQvC3"

// ==========================================
// KONFIGURASI PIN MOTOR, SERVO 360 & INDIKATOR
// ==========================================
#define DIR_PIN_1 27  // Motor 1: Sumbu X (Horizontal)
#define STEP_PIN_1 14 
#define DIR_PIN_2 25  // Motor 2: Sumbu Z (Vertikal)
#define STEP_PIN_2 26 

#define SERVO_PIN 32  // Pin PWM Servo 360 Derajat
#define SERVO_LEDC_CHANNEL 0
#define SERVO_PWM_FREQ 50
#define SERVO_PWM_RES_BITS 14 // 14-bit (0..16383) kompatibel di semua ESP32 pada 50Hz

#define GLOBAL_EN_PIN 13 
#define INDICATOR_PIN_1 15 
#define INDICATOR_PIN_2 2  

// ==========================================
// KONFIGURASI MICROSTEPPING PINS
// ==========================================
#define MS1_PIN 4
#define MS2_PIN 5
#define MS3_PIN 17

// ==========================================
// KONFIGURASI LIMIT SWITCH (Internal Pull-up)
// ==========================================
#define LS_X_LEFT 18
#define LS_X_RIGHT 19
#define LS_Z_TOP 21
#define LS_Z_BOTTOM 22

#define ENABLE_SAFETY_LIMIT_SWITCHES false

// ==========================================
// PARAMETER MOTOR & KALIBRASI RAK (BOX ADDRESS)
// ==========================================
const float stepsPerRevolution = 200.0f;
const int microstepSetting = 1; // 1 = Full step, 8 = 1/8 step

float targetRPM = 60.0f;
float targetAccelStepsPerSec2 = 800.0f;

// =========================================================================
// TABEL POSISI PUTARAN (REVOLUTIONS) DARI [0,0] KE TIAP BOX [A..C][1..3]
// =========================================================================
float BOX_X_REV[3][3] = {
  { 2.0f, 5.0f, 8.0f }, // Baris A: A1, A2, A3
  { 2.0f, 5.0f, 8.0f }, // Baris B: B1, B2, B3
  { 2.0f, 5.0f, 8.0f }  // Baris C: C1, C2, C3
};
float BOX_Z_REV[3][3] = {
  { 1.5f, 1.5f, 1.5f }, // Baris A: A1, A2, A3
  { 4.0f, 4.0f, 4.0f }, // Baris B: B1, B2, B3
  { 6.5f, 6.5f, 6.5f }  // Baris C: C1, C2, C3
};

// Offset gerak sumbu Z (default 0.4 putaran) setelah Servo Extend & sebelum Servo Retract
float zLiftOffsetRev = 0.40f;

// Konfigurasi Timing & PWM Servo 360 Derajat (50Hz, 14-bit resolution)
const int SERVO_PULSE_STOP    = 1500;
const int SERVO_PULSE_EXTEND  = 2000; 
const int SERVO_PULSE_RETRACT = 1000;

float servoExtractRev = 1.5f;
float servoRetractRev = 1.5f;
unsigned long servoMsPerRev = 1000;
unsigned long SERVO_EXTEND_MS  = 1500;
unsigned long SERVO_RETRACT_MS = 1500;

bool isManualServoTimed = false;
unsigned long manualServoStartMs = 0;
unsigned long manualServoTargetMs = 0;

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1); // X-Axis
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2); // Z-Axis

FirebaseData fbData;
FirebaseAuth auth;
FirebaseConfig config;
Preferences prefs;

RunMode currentMode = MODE_IDLE;
AsrsStep asrsState = ASRS_NONE;
bool isMotorRunning = false; 
bool isSystemLocked = false; 

char activeBoxAddress[8] = "NONE";
char activeOperation[16] = "NONE";
char currentServoState[12] = "STOP";
float targetXRev = 0.0f;
float targetZRev = 0.0f;
unsigned long servoTimerStart = 0;

QueueHandle_t commandQueue = NULL;
QueueHandle_t restoreQueue = NULL;
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
TelemetrySnapshot sharedTelemetry;

volatile bool pendingFirebaseSync = false;
volatile bool pendingConfigSave   = false;
volatile bool pendingConfigLoad   = true;

// Konversi state internal ke nomor stage 1..6 untuk UI Website
int getUiStepStage(AsrsStep st) {
  if (st == ASRS_STEP_5A_SETTLE) return 3;
  if (st == ASRS_STEP_5C_SETTLE) return 5;
  if ((int)st >= 1 && (int)st <= 6) return (int)st;
  return 0;
}

// =========================================================================
// SIMPAN & MUAT KONFIGURASI LOKAL (ESP32 NVS FLASH)
// =========================================================================
void saveStateToNVS() {
  prefs.begin("asrs_cfg", false);
  prefs.putLong("m1_steps", stepper1.currentPosition());
  prefs.putLong("m2_steps", stepper2.currentPosition());
  prefs.putFloat("rpm", targetRPM);
  prefs.putFloat("accel", targetAccelStepsPerSec2);
  prefs.putFloat("s_ext", servoExtractRev);
  prefs.putFloat("s_ret", servoRetractRev);
  prefs.putULong("s_ms", servoMsPerRev);
  prefs.putFloat("z_off", zLiftOffsetRev);
  prefs.putBytes("box_x", BOX_X_REV, sizeof(BOX_X_REV));
  prefs.putBytes("box_z", BOX_Z_REV, sizeof(BOX_Z_REV));
  prefs.putBool("init", true);
  prefs.end();
}

void loadStateFromNVS() {
  prefs.begin("asrs_cfg", true);
  if (prefs.getBool("init", false)) {
    long m1 = prefs.getLong("m1_steps", 0);
    long m2 = prefs.getLong("m2_steps", 0);
    stepper1.setCurrentPosition(m1);
    stepper2.setCurrentPosition(m2);

    targetRPM = prefs.getFloat("rpm", 60.0f);
    targetAccelStepsPerSec2 = prefs.getFloat("accel", 800.0f);
    servoExtractRev = prefs.getFloat("s_ext", 1.5f);
    servoRetractRev = prefs.getFloat("s_ret", 1.5f);
    servoMsPerRev = prefs.getULong("s_ms", 1000);
    zLiftOffsetRev = prefs.getFloat("z_off", 0.40f);

    if (prefs.getBytesLength("box_x") == sizeof(BOX_X_REV)) {
      prefs.getBytes("box_x", BOX_X_REV, sizeof(BOX_X_REV));
    }
    if (prefs.getBytesLength("box_z") == sizeof(BOX_Z_REV)) {
      prefs.getBytes("box_z", BOX_Z_REV, sizeof(BOX_Z_REV));
    }
    Serial.printf("[NVS FLASH] Dipulihkan: X=%ld st, Z=%ld st, RPM=%.1f, Z-Offset=%.2f\n",
                  m1, m2, targetRPM, zLiftOffsetRev);
  }
  prefs.end();
}

// =========================================================================
// SINKRONISASI TELEMETRI & KONFIGURASI ONLINE (THREAD-SAFE)
// =========================================================================
void requestFirebaseSync(const char* statusMsg, bool saveConfigOnline) {
  float denom = stepsPerRevolution * (float)microstepSetting;
  long pos1 = stepper1.currentPosition();
  long pos2 = stepper2.currentPosition();

  portENTER_CRITICAL(&telemetryMux);
  strncpy(sharedTelemetry.statusMsg, statusMsg, sizeof(sharedTelemetry.statusMsg) - 1);
  sharedTelemetry.statusMsg[sizeof(sharedTelemetry.statusMsg) - 1] = '\0';
  strncpy(sharedTelemetry.box, activeBoxAddress, sizeof(sharedTelemetry.box) - 1);
  strncpy(sharedTelemetry.op, activeOperation, sizeof(sharedTelemetry.op) - 1);
  strncpy(sharedTelemetry.servoState, currentServoState, sizeof(sharedTelemetry.servoState) - 1);

  sharedTelemetry.locked = isSystemLocked;
  sharedTelemetry.running = isMotorRunning;
  sharedTelemetry.m1Steps = pos1;
  sharedTelemetry.m2Steps = pos2;
  sharedTelemetry.m1Rev = (float)pos1 / denom;
  sharedTelemetry.m2Rev = (float)pos2 / denom;
  sharedTelemetry.servoExtRev = servoExtractRev;
  sharedTelemetry.servoRetRev = servoRetractRev;
  sharedTelemetry.servoMsRev = servoMsPerRev;
  sharedTelemetry.stepStage = getUiStepStage(asrsState);
  sharedTelemetry.rpm = targetRPM;
  sharedTelemetry.accel = targetAccelStepsPerSec2;
  sharedTelemetry.zOffsetRev = zLiftOffsetRev;
  memcpy(sharedTelemetry.boxX, BOX_X_REV, sizeof(BOX_X_REV));
  memcpy(sharedTelemetry.boxZ, BOX_Z_REV, sizeof(BOX_Z_REV));

  pendingFirebaseSync = true;
  if (saveConfigOnline) {
    pendingConfigSave = true;
  }
  portEXIT_CRITICAL(&telemetryMux);

  if (saveConfigOnline) {
    saveStateToNVS();
  }
}

void requestFirebaseSync(const String& statusMsg, bool saveConfigOnline) {
  requestFirebaseSync(statusMsg.c_str(), saveConfigOnline);
}

// ==========================================
// FUNGSI KONTROL SERVO 360 DERAJAT (PWM 50Hz, 14-BIT)
// ==========================================
void writeServoMicroseconds(int us) {
  uint32_t maxDuty = (1UL << SERVO_PWM_RES_BITS); // 16384
  uint32_t duty = ((uint32_t)us * maxDuty) / 20000UL;

  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(SERVO_PIN, duty);
  #else
    ledcWrite(SERVO_LEDC_CHANNEL, duty);
  #endif
}

void setServo360(const char* action) {
  strncpy(currentServoState, action, sizeof(currentServoState) - 1);
  currentServoState[sizeof(currentServoState) - 1] = '\0';

  if (strcmp(action, "EXTEND") == 0) {
    writeServoMicroseconds(SERVO_PULSE_EXTEND);
    Serial.printf("[SERVO PWM] EXTEND (%d us)\n", SERVO_PULSE_EXTEND);
  } else if (strcmp(action, "RETRACT") == 0) {
    writeServoMicroseconds(SERVO_PULSE_RETRACT);
    Serial.printf("[SERVO PWM] RETRACT (%d us)\n", SERVO_PULSE_RETRACT);
  } else {
    writeServoMicroseconds(SERVO_PULSE_STOP);
    Serial.printf("[SERVO PWM] STOP (%d us)\n", SERVO_PULSE_STOP);
  }
}

void recalculateServoTimings() {
  if (servoExtractRev < 0.05f) servoExtractRev = 0.05f;
  if (servoRetractRev < 0.05f) servoRetractRev = 0.05f;
  if (servoMsPerRev < 100) servoMsPerRev = 100;

  SERVO_EXTEND_MS  = (unsigned long)round(servoExtractRev * (float)servoMsPerRev);
  SERVO_RETRACT_MS = (unsigned long)round(servoRetractRev * (float)servoMsPerRev);
}

void startManualServoRevolutions(const char* direction, float revolutions) {
  if (isSystemLocked) {
    Serial.println("[DITOLAK] Sistem terkunci! Kirim STORAGE / UNLOCK dulu.");
    return;
  }
  recalculateServoTimings();
  if (revolutions <= 0.0f) {
    isManualServoTimed = false;
    setServo360("STOP");
    return;
  }
  manualServoTargetMs = (unsigned long)round(revolutions * (float)servoMsPerRev);
  manualServoStartMs = millis();
  isManualServoTimed = true;
  setServo360(direction);
  Serial.printf(">> [SERVO 360 TIMED] %s sejauh %.2f Rev (%lu ms)...\n",
                direction, revolutions, manualServoTargetMs);
}

void applySpeedAndRampSettings() {
  float maxSpeedStepsPerSec = (microstepSetting * stepsPerRevolution * targetRPM) / 60.0f;
  if (maxSpeedStepsPerSec < 10.0f) maxSpeedStepsPerSec = 10.0f;

  stepper1.setMaxSpeed(maxSpeedStepsPerSec);
  stepper2.setMaxSpeed(maxSpeedStepsPerSec);

  stepper1.setAcceleration(targetAccelStepsPerSec2);
  stepper2.setAcceleration(targetAccelStepsPerSec2);

  if (isMotorRunning && currentMode == MODE_CONTINUOUS) {
    stepper1.setSpeed(maxSpeedStepsPerSec);
    stepper2.setSpeed(maxSpeedStepsPerSec);
  }
}

// ==========================================
// FUNGSI BANTUAN & KEAMANAN
// ==========================================
bool isLimitSwitchHit() {
  #if ENABLE_SAFETY_LIMIT_SWITCHES
    return (digitalRead(LS_X_LEFT) == LOW ||
            digitalRead(LS_Z_TOP) == LOW ||
            digitalRead(LS_Z_BOTTOM) == LOW);
  #else
    return false;
  #endif
}

bool isHomingLimitSwitchPressed() {
  return (digitalRead(LS_X_RIGHT) == LOW);
}

void stopMotorsNormal() {
  isMotorRunning = false;
  currentMode = MODE_IDLE;
  asrsState = ASRS_NONE;
  isManualServoTimed = false;

  stepper1.setSpeed(0);
  stepper2.setSpeed(0);

  stepper1.moveTo(stepper1.currentPosition());
  stepper2.moveTo(stepper2.currentPosition());

  setServo360("STOP");

  digitalWrite(INDICATOR_PIN_1, LOW);
  digitalWrite(INDICATOR_PIN_2, LOW);
}

void triggerEmergencyStop() {
  isSystemLocked = true;  
  stopMotorsNormal();
}

bool resolveBoxCoordinates(const String& boxAddr, float &outXRev, float &outZRev) {
  if (boxAddr.length() < 2) return false;

  int rowIdx = boxAddr.charAt(0) - 'A';
  int colIdx = boxAddr.charAt(1) - '1';

  if (rowIdx < 0 || rowIdx > 2 || colIdx < 0 || colIdx > 2) return false;

  outXRev = BOX_X_REV[rowIdx][colIdx];
  outZRev = BOX_Z_REV[rowIdx][colIdx];
  return true;
}

void startAsrsCycle(const char* opType, const String& boxAddr, float customXRev, float customZRev) {
  if (isSystemLocked) {
    Serial.println("[DITOLAK] Sistem terkunci! Kirim 'STORAGE' / 'RESET' dulu.");
    requestFirebaseSync("REJECTED: SYSTEM LOCKED");
    return;
  }

  if (isMotorRunning && currentMode == MODE_ASRS_SEQUENCE) {
    Serial.println("[DITOLAK] Siklus ASRS sedang berjalan! Tunggu selesai atau kirim STOP.");
    requestFirebaseSync("BUSY: ASRS CYCLE IN PROGRESS");
    return;
  }

  float xRev = 0.0f, zRev = 0.0f;
  if (customXRev >= 0.0f && customZRev >= 0.0f) {
    xRev = customXRev;
    zRev = customZRev;
    if (boxAddr.length() >= 2) {
      int r = boxAddr.charAt(0) - 'A';
      int c = boxAddr.charAt(1) - '1';
      if (r >= 0 && r <= 2 && c >= 0 && c <= 2) {
        BOX_X_REV[r][c] = xRev;
        BOX_Z_REV[r][c] = zRev;
      }
    }
  } else if (!resolveBoxCoordinates(boxAddr, xRev, zRev)) {
    Serial.println("[ERROR] Alamat Box tidak valid! Gunakan A1..C3.");
    requestFirebaseSync("INVALID BOX ADDRESS");
    return;
  }

  strncpy(activeOperation, opType, sizeof(activeOperation) - 1);
  activeOperation[sizeof(activeOperation) - 1] = '\0';
  strncpy(activeBoxAddress, boxAddr.c_str(), sizeof(activeBoxAddress) - 1);
  activeBoxAddress[sizeof(activeBoxAddress) - 1] = '\0';

  targetXRev = xRev;
  targetZRev = zRev;

  recalculateServoTimings();
  applySpeedAndRampSettings();
  digitalWrite(GLOBAL_EN_PIN, LOW);
  digitalWrite(INDICATOR_PIN_1, HIGH);
  digitalWrite(INDICATOR_PIN_2, HIGH);

  long targetXSteps = round(targetXRev * stepsPerRevolution * microstepSetting);
  stepper1.moveTo(targetXSteps);

  currentMode = MODE_ASRS_SEQUENCE;
  asrsState = ASRS_STEP_3_MOVE_X;
  isMotorRunning = true;

  Serial.println("\n================ MEMULAI SIKLUS OTOMATIS ASRS ================");
  Serial.printf("1. Box Address : %s\n", activeBoxAddress);
  Serial.printf("2. Operation   : %s\n", activeOperation);
  Serial.printf("3. [PROC] Moving X-Axis to %.2f Rev (%ld steps)...\n", targetXRev, targetXSteps);
  Serial.println("==============================================================");

  char msg[64];
  snprintf(msg, sizeof(msg), "STEP 3: MOVING X -> %s (%s)", activeBoxAddress, activeOperation);
  requestFirebaseSync(msg);
}

void moveRevolutions(int motorID, float revolutions) {
  if (isSystemLocked) {
    Serial.println("[DITOLAK] Sistem terkunci! Ketik 'RESET' atau kirim 'STORAGE'.");
    requestFirebaseSync("REJECTED: SYSTEM LOCKED");
    return;
  }

  if (isMotorRunning && currentMode == MODE_ASRS_SEQUENCE) {
    Serial.println("[DITOLAK] Tidak bisa gerak manual saat siklus ASRS berjalan!");
    return;
  }

  long targetSteps = round(revolutions * stepsPerRevolution * microstepSetting);
  if (targetSteps == 0) {
    stopMotorsNormal();
    return;
  }

  applySpeedAndRampSettings();
  digitalWrite(GLOBAL_EN_PIN, LOW);
  digitalWrite(INDICATOR_PIN_1, HIGH);
  digitalWrite(INDICATOR_PIN_2, HIGH);

  if (motorID == 0 || motorID == 1) stepper1.move(targetSteps);
  if (motorID == 0 || motorID == 2) stepper2.move(targetSteps);

  currentMode = MODE_POSITION;
  asrsState = ASRS_NONE;
  isMotorRunning = true;

  Serial.printf(">> Manual Move Motor %d: %.2f rev (%ld steps)\n", motorID, revolutions, targetSteps);
}

// ==========================================
// EKSEKUSI PERINTAH (HANYA DIJALANKAN DI CORE 1!)
// ==========================================
void printSerialHelp() {
  Serial.println("\n================ PANDUAN KONTROL ASRS (SERIAL & WEBSITE) ================");
  Serial.println("A. SIKLUS OTOMATIS ASRS (6 LANGKAH):");
  Serial.println("   - 'STORE:A1' atau 'RETRIEVE:B3' -> Siklus: X -> Z -> Servo Extend -> Z Offset -> Servo Retract -> Home.");
  Serial.println("   - 'STORE:C2:5.0:6.5'            -> Siklus dengan kustom putaran X=5.0, Z=6.5.");
  Serial.println("B. SETTING KOORDINAT PUTARAN BOX, Z-OFFSET & CLOUD BACKUP:");
  Serial.println("   - 'SETBOX:B2:5.5:4.2'           -> Set & simpan online koordinat Box B2 dari [0,0].");
  Serial.println("   - 'ZOFFSET:0.40'                -> Set & simpan online Z Lift/Drop Offset (rev).");
  Serial.println("   - 'TESTSLOT:A1:2.0:1.5'         -> Uji gerak X & Z ke slot tanpa menggerakkan servo.");
  Serial.println("   - 'SAVE' / 'LOAD'               -> Simpan / muat ulang semua setting & posisi dari Firebase.");
  Serial.println("C. KONTROL & SETTING PUTARAN SERVO 360 DERAJAT (PIN 32):");
  Serial.println("   - 'SERVOCFG:<extRev>:<retRev>:<msPerRev>' -> Set & simpan putaran Extract, Retract & ms/rev.");
  Serial.println("   - 'SERVO:EXTEND_REV' / 'SERVO:RETRACT_REV'-> Uji putar servo sesuai setting Rev.");
  Serial.println("D. KONTROL MANUAL MOTOR X (M1) & MOTOR Z (M2):");
  Serial.println("   - 'M1:3.5' / 'M2:2.0' | 'SPEED:<rpm>' | 'ACCEL:<val>' | 'HOME' | 'ZERO'");
  Serial.println("=========================================================================\n");
}

void executeUnifiedCommand(String cmd, const char* source) {
  cmd.replace("\"", "");
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0 || cmd == "NONE") return;

  Serial.printf("[%s] Perintah dieksekusi di Core %d: %s\n", source, xPortGetCoreID(), cmd.c_str());

  if (cmd == "HELP" || cmd == "?") {
    printSerialHelp();
  }
  // 1. EMERGENCY STOP
  else if (cmd == "E_STOP" || cmd == "E-STOP" || cmd == "ESTOP") {
    Serial.println("EMERGENCY STOP - SISTEM TERKUNCI!");
    triggerEmergencyStop();
    digitalWrite(GLOBAL_EN_PIN, LOW);
    requestFirebaseSync("EMERGENCY STOP (LOCKED)", true);
  }
  // 2. RESET KUNCI KEAMANAN (STORAGE / RESET / UNLOCK)
  else if (cmd == "STORAGE" || cmd == "RESET" || cmd == "UNLOCK") {
    isSystemLocked = false;
    Serial.println("Sistem Di-reset (UNLOCK). Keamanan dibuka.");
    requestFirebaseSync("IDLE (UNLOCKED)", true);
  }
  // 3. STOP NORMAL (RETRIEVAL / STOP)
  else if (cmd == "RETRIEVAL" || cmd == "STOP" || cmd == "S") {
    stopMotorsNormal();
    Serial.println("Menghentikan putaran motor & servo secara normal.");
    requestFirebaseSync("STOPPED", true);
  }
  // 3B. SIMPAN / MUAT KONFIGURASI & POSISI DARI FIREBASE CLOUD
  else if (cmd == "SAVE") {
    Serial.println(">> [CLOUD SYNC] Menyimpan seluruh setting & posisi ke Firebase & NVS...");
    requestFirebaseSync("SETTINGS & POS SAVED ONLINE", true);
  }
  else if (cmd == "LOAD") {
    if (!isMotorRunning) {
      Serial.println(">> [CLOUD SYNC] Meminta pemulihan setting & posisi dari Firebase...");
      pendingConfigLoad = true;
    } else {
      Serial.println("[DITOLAK] Tidak dapat memuat konfigurasi saat motor sedang bergerak!");
    }
  }
  // 4. PERINTAH SIKLUS OTOMATIS ASRS
  else if (cmd.startsWith("STORE:") || cmd.startsWith("RETRIEVE:")) {
    int firstColon = cmd.indexOf(':');
    String opType = cmd.substring(0, firstColon);
    String rest = cmd.substring(firstColon + 1);

    String boxAddr = rest;
    float customX = -1.0f, customZ = -1.0f;

    int secondColon = rest.indexOf(':');
    if (secondColon != -1) {
      boxAddr = rest.substring(0, secondColon);
      String coords = rest.substring(secondColon + 1);
      int thirdColon = coords.indexOf(':');
      if (thirdColon != -1) {
        customX = coords.substring(0, thirdColon).toFloat();
        customZ = coords.substring(thirdColon + 1).toFloat();
      }
    }
    startAsrsCycle(opType == "STORE" ? "STORAGE" : "RETRIEVAL", boxAddr, customX, customZ);
  }
  // 4B. KALIBRASI SPESIFIK PER BOX DARI [0,0] (Otomatis Disimpan Online!)
  else if (cmd.startsWith("SETBOX:")) {
    String params = cmd.substring(7);
    int c1 = params.indexOf(':');
    int c2 = params.indexOf(':', c1 + 1);
    if (c1 != -1 && c2 != -1) {
      String addr = params.substring(0, c1);
      float xVal = params.substring(c1 + 1, c2).toFloat();
      float zVal = params.substring(c2 + 1).toFloat();
      if (addr.length() >= 2) {
        int r = addr.charAt(0) - 'A';
        int c = addr.charAt(1) - '1';
        if (r >= 0 && r <= 2 && c >= 0 && c <= 2) {
          BOX_X_REV[r][c] = xVal;
          BOX_Z_REV[r][c] = zVal;
          Serial.printf(">> [KALIBRASI BOX %s] Disimpan Online -> X: %.2f rev, Z: %.2f rev\n",
                        addr.c_str(), xVal, zVal);
          requestFirebaseSync("BOX " + addr + " SAVED ONLINE", true);
        }
      }
    }
  }
  // 4C. KALIBRASI Z-OFFSET DARI WEBSITE: ZOFFSET:<val>
  else if (cmd.startsWith("ZOFFSET:")) {
    float val = cmd.substring(8).toFloat();
    if (val != 0.0f && val >= -5.0f && val <= 5.0f) {
      zLiftOffsetRev = val;
      Serial.printf(">> [SETTING Z-OFFSET] Disimpan Online -> %.2f rev\n", zLiftOffsetRev);
      requestFirebaseSync("Z-OFFSET SET: " + String(zLiftOffsetRev, 2) + " REV", true);
    }
  }
  // 4D. UJI GERAK KE SLOT TANPA SERVO DARI WEBSITE: TESTSLOT:<SLOT>:<X_REV>:<Z_REV>
  else if (cmd.startsWith("TESTSLOT:")) {
    if (isSystemLocked) {
      requestFirebaseSync("REJECTED: SYSTEM LOCKED");
      return;
    }
    String params = cmd.substring(9);
    int c1 = params.indexOf(':');
    int c2 = params.indexOf(':', c1 + 1);
    String addr = params;
    float xRev = 0.0f, zRev = 0.0f;

    if (c1 != -1 && c2 != -1) {
      addr = params.substring(0, c1);
      xRev = params.substring(c1 + 1, c2).toFloat();
      zRev = params.substring(c2 + 1).toFloat();
    } else if (!resolveBoxCoordinates(addr, xRev, zRev)) {
      requestFirebaseSync("INVALID TEST SLOT");
      return;
    }

    strncpy(activeBoxAddress, addr.c_str(), sizeof(activeBoxAddress) - 1);
    activeBoxAddress[sizeof(activeBoxAddress) - 1] = '\0';
    strcpy(activeOperation, "TEST");

    applySpeedAndRampSettings();
    digitalWrite(GLOBAL_EN_PIN, LOW);
    digitalWrite(INDICATOR_PIN_1, HIGH);
    digitalWrite(INDICATOR_PIN_2, HIGH);

    long targetXSteps = round(xRev * stepsPerRevolution * microstepSetting);
    long targetZSteps = round(zRev * stepsPerRevolution * microstepSetting);
    stepper1.moveTo(targetXSteps);
    stepper2.moveTo(targetZSteps);

    currentMode = MODE_POSITION;
    asrsState = ASRS_NONE;
    isMotorRunning = true;
    Serial.printf(">> [TEST SLOT %s] Menggerakkan X ke %.2f rev & Z ke %.2f rev...\n", addr.c_str(), xRev, zRev);
    requestFirebaseSync("TESTING SLOT " + addr);
  }
  // 5. KONTROL & KALIBRASI PUTARAN SERVO 360 DERAJAT (Otomatis Disimpan Online!)
  else if (cmd.startsWith("SERVOCFG:")) {
    String params = cmd.substring(9);
    int c1 = params.indexOf(':');
    int c2 = params.indexOf(':', c1 + 1);
    if (c1 != -1 && c2 != -1) {
      float extR = params.substring(0, c1).toFloat();
      float retR = params.substring(c1 + 1, c2).toFloat();
      long msRev = params.substring(c2 + 1).toInt();

      if (extR > 0.0f && retR > 0.0f && msRev >= 100 && msRev <= 20000) {
        servoExtractRev = extR;
        servoRetractRev = retR;
        servoMsPerRev = (unsigned long)msRev;
        recalculateServoTimings();
        Serial.printf(">> [SETTING SERVO 360] Disimpan Online -> Extract: %.2f rev (%lu ms) | Retract: %.2f rev (%lu ms)\n",
                      servoExtractRev, SERVO_EXTEND_MS, servoRetractRev, SERVO_RETRACT_MS);
        requestFirebaseSync("SERVO CFG SAVED ONLINE", true);
      }
    }
  }
  else if (cmd.startsWith("SERVOEXTREV:")) {
    float extR = cmd.substring(12).toFloat();
    if (extR > 0.0f && extR <= 100.0f) {
      servoExtractRev = extR;
      recalculateServoTimings();
      requestFirebaseSync("SERVO EXT REV: " + String(servoExtractRev, 2), true);
    }
  }
  else if (cmd.startsWith("SERVORETREV:")) {
    float retR = cmd.substring(12).toFloat();
    if (retR > 0.0f && retR <= 100.0f) {
      servoRetractRev = retR;
      recalculateServoTimings();
      requestFirebaseSync("SERVO RET REV: " + String(servoRetractRev, 2), true);
    }
  }
  else if (cmd.startsWith("SERVO:")) {
    String act = cmd.substring(6);
    if (act == "EXTEND_REV") {
      startManualServoRevolutions("EXTEND", servoExtractRev);
      requestFirebaseSync("SERVO EXTENDING");
    } else if (act == "RETRACT_REV") {
      startManualServoRevolutions("RETRACT", servoRetractRev);
      requestFirebaseSync("SERVO RETRACTING");
    } else if (act.startsWith("EXTEND:")) {
      float customRev = act.substring(7).toFloat();
      startManualServoRevolutions("EXTEND", customRev);
      requestFirebaseSync("SERVO EXTENDING");
    } else if (act.startsWith("RETRACT:")) {
      float customRev = act.substring(8).toFloat();
      startManualServoRevolutions("RETRACT", customRev);
      requestFirebaseSync("SERVO RETRACTING");
    } else {
      isManualServoTimed = false;
      setServo360(act.c_str());
      requestFirebaseSync("SERVO: " + act);
    }
  }
  else if (cmd.startsWith("SERVOTIME:")) {
    long ms = cmd.substring(10).toInt();
    if (ms >= 100 && ms <= 20000) {
      servoMsPerRev = (unsigned long)ms;
      recalculateServoTimings();
      requestFirebaseSync("SERVO MS/REV: " + String(servoMsPerRev), true);
    }
  }
  // 6. KEMBALI KE HOME (0, 0) DENGAN RAMP
  else if (cmd == "HOME") {
    if (!isSystemLocked) {
      applySpeedAndRampSettings();
      digitalWrite(GLOBAL_EN_PIN, LOW);
      digitalWrite(INDICATOR_PIN_1, HIGH);
      digitalWrite(INDICATOR_PIN_2, HIGH);
      stepper1.moveTo(0);
      stepper2.moveTo(0);
      currentMode = MODE_POSITION;
      asrsState = ASRS_NONE;
      isMotorRunning = true;
      Serial.println(">> Bergerak kembali ke Posisi HOME (0, 0)...");
      requestFirebaseSync("RETURNING TO HOME (0,0)");
    }
  }
  // 7. PUTAR TANPA BATAS (HOMING)
  else if (cmd == "HOMING") {
    if (isSystemLocked) {
      requestFirebaseSync("REJECTED: SYSTEM LOCKED");
    } else {
      digitalWrite(GLOBAL_EN_PIN, LOW);
      digitalWrite(INDICATOR_PIN_1, HIGH);
      digitalWrite(INDICATOR_PIN_2, HIGH);

      applySpeedAndRampSettings();
      float speedStepsPerSec = (microstepSetting * stepsPerRevolution * targetRPM) / 60.0f;
      stepper1.setSpeed(speedStepsPerSec);
      stepper2.setSpeed(speedStepsPerSec);

      currentMode = MODE_CONTINUOUS;
      isMotorRunning = true;
      requestFirebaseSync("HOMING (CONTINUOUS SPIN)");
    }
  }
  // 8. PENGATURAN KECEPATAN & RAMP AKSELERASI (Otomatis Disimpan Online!)
  else if (cmd.startsWith("SPEED:") || cmd.startsWith("RPM:")) {
    int colonIdx = cmd.indexOf(':');
    float newRPM = cmd.substring(colonIdx + 1).toFloat();
    if (newRPM >= 1.0f && newRPM <= 600.0f) {
      targetRPM = newRPM;
      applySpeedAndRampSettings();
      requestFirebaseSync("SPEED SET: " + String(targetRPM, 0) + " RPM", true);
    }
  }
  else if (cmd.startsWith("ACCEL:") || cmd.startsWith("RAMP:")) {
    int colonIdx = cmd.indexOf(':');
    float newAccel = cmd.substring(colonIdx + 1).toFloat();
    if (newAccel >= 10.0f && newAccel <= 20000.0f) {
      targetAccelStepsPerSec2 = newAccel;
      applySpeedAndRampSettings();
      requestFirebaseSync("RAMP SET: " + String(targetAccelStepsPerSec2, 0), true);
    }
  }
  // 9. RESET POSISI KE 0 (ZERO - Otomatis Disimpan Online!)
  else if (cmd == "ZERO") {
    if (!isMotorRunning) {
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      Serial.println("Posisi Motor X & Z di-reset ke 0 (Titik Home) & Disimpan Online.");
      requestFirebaseSync("POSITION ZEROED (SAVED)", true);
    }
  }
  // 10. CEK POSISI SAAT INI (POS)
  else if (cmd == "POS") {
    requestFirebaseSync("POSITION QUERIED", true);
  }
  // 11. MOTOR SPESIFIK (M1:<rev> untuk X, M2:<rev> untuk Z)
  else if (cmd.startsWith("M1:") || cmd.startsWith("X:")) {
    float revs = cmd.substring(cmd.indexOf(':') + 1).toFloat();
    moveRevolutions(1, revs);
    requestFirebaseSync("MOVING X: " + String(revs, 2) + " REV");
  }
  else if (cmd.startsWith("M2:") || cmd.startsWith("Z:")) {
    float revs = cmd.substring(cmd.indexOf(':') + 1).toFloat();
    moveRevolutions(2, revs);
    requestFirebaseSync("MOVING Z: " + String(revs, 2) + " REV");
  }
  // 12. ANGKA LANGSUNG
  else {
    String numStr = cmd;
    if (numStr.startsWith("REV:")) {
      numStr = numStr.substring(4);
      numStr.trim();
    }

    bool isNumeric = (numStr.length() > 0);
    for (unsigned int i = 0; i < numStr.length(); i++) {
      char c = numStr.charAt(i);
      if (!isDigit(c) && c != '.' && c != '-' && c != '+') {
        isNumeric = false;
        break;
      }
    }

    if (isNumeric) {
      float revs = numStr.toFloat();
      moveRevolutions(0, revs);
      requestFirebaseSync("MOVING X & Z: " + String(revs, 2) + " REV");
    } else {
      Serial.print("[ERROR] Perintah tidak dikenali: ");
      Serial.println(cmd);
    }
  }
}

// ==========================================
// PEMBACAAN SERIAL, COMMAND QUEUE & RESTORE QUEUE DI CORE 1
// ==========================================
char serialBuf[64];
uint8_t serialIdx = 0;

void applyRestoredConfigOnCore1(const RestoredConfigPacket& cfg) {
  if (!cfg.valid) return;

  if (!isMotorRunning && cfg.hasSteps) {
    stepper1.setCurrentPosition(cfg.m1Steps);
    stepper2.setCurrentPosition(cfg.m2Steps);
  }

  targetRPM = cfg.rpm;
  targetAccelStepsPerSec2 = cfg.accel;
  servoExtractRev = cfg.servoExtRev;
  servoRetractRev = cfg.servoRetRev;
  servoMsPerRev = cfg.servoMsRev;
  zLiftOffsetRev = cfg.zOffsetRev;

  memcpy(BOX_X_REV, cfg.boxX, sizeof(BOX_X_REV));
  memcpy(BOX_Z_REV, cfg.boxZ, sizeof(BOX_Z_REV));

  recalculateServoTimings();
  applySpeedAndRampSettings();
  saveStateToNVS();

  float denom = stepsPerRevolution * (float)microstepSetting;
  Serial.println("\n================ [CLOUD RESTORE BERHASIL] ================");
  Serial.printf("-> Posisi Motor X (M1) : %ld steps (%.2f rev)\n", stepper1.currentPosition(), (float)stepper1.currentPosition() / denom);
  Serial.printf("-> Posisi Motor Z (M2) : %ld steps (%.2f rev)\n", stepper2.currentPosition(), (float)stepper2.currentPosition() / denom);
  Serial.printf("-> Speed & Ramp        : %.1f RPM | %.0f steps/s^2\n", targetRPM, targetAccelStepsPerSec2);
  Serial.printf("-> Z Lift/Drop Offset  : %.2f rev\n", zLiftOffsetRev);
  Serial.printf("-> Servo 360 Config    : Ext %.2f rev (%lu ms) | Ret %.2f rev (%lu ms) @ %lu ms/rev\n",
                servoExtractRev, SERVO_EXTEND_MS, servoRetractRev, SERVO_RETRACT_MS, servoMsPerRev);
  Serial.printf("-> Koordinat Box A1..C3: A1(%.1f, %.1f), B2(%.1f, %.1f), C3(%.1f, %.1f)\n",
                BOX_X_REV[0][0], BOX_Z_REV[0][0], BOX_X_REV[1][1], BOX_Z_REV[1][1], BOX_X_REV[2][2], BOX_Z_REV[2][2]);
  Serial.println("==========================================================\n");

  requestFirebaseSync("ONLINE SETTINGS & POS RESTORED", true);
}

void checkSerialAndQueueInput() {
  RestoredConfigPacket restoredCfg;
  while (xQueueReceive(restoreQueue, &restoredCfg, 0) == pdTRUE) {
    applyRestoredConfigOnCore1(restoredCfg);
  }

  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (serialIdx > 0) {
        serialBuf[serialIdx] = '\0';
        executeUnifiedCommand(String(serialBuf), "SERIAL");
        serialIdx = 0;
      }
    } else if (serialIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialIdx++] = inChar;
    }
  }

  CommandPacket pkt;
  while (xQueueReceive(commandQueue, &pkt, 0) == pdTRUE) {
    executeUnifiedCommand(String(pkt.cmd), pkt.source);
  }
}

// ==========================================
// TUGAS CORE 0: FIREBASE CLOUD BACKUP, RESTORE & POLLING
// ==========================================
void saveConfigAndPositionToFirebase(const TelemetrySnapshot& snap) {
  FirebaseJson cfgJson;
  cfgJson.set("m1_steps", (int)snap.m1Steps);
  cfgJson.set("m2_steps", (int)snap.m2Steps);
  cfgJson.set("m1_rev", snap.m1Rev);
  cfgJson.set("m2_rev", snap.m2Rev);
  cfgJson.set("rpm", snap.rpm);
  cfgJson.set("accel", snap.accel);
  cfgJson.set("servo_ext_rev", snap.servoExtRev);
  cfgJson.set("servo_ret_rev", snap.servoRetRev);
  cfgJson.set("servo_ms_rev", (int)snap.servoMsRev);
  cfgJson.set("z_offset", snap.zOffsetRev);

  const char rows[3] = {'A', 'B', 'C'};
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      char pathX[32], pathZ[32];
      snprintf(pathX, sizeof(pathX), "coordinates/%c%d/x", rows[r], c + 1);
      snprintf(pathZ, sizeof(pathZ), "coordinates/%c%d/z", rows[r], c + 1);
      cfgJson.set(pathX, snap.boxX[r][c]);
      cfgJson.set(pathZ, snap.boxZ[r][c]);
    }
  }

  Firebase.updateNodeSilent(fbData, "/asrs/config", cfgJson);
}

bool loadConfigAndPositionFromFirebase() {
  RestoredConfigPacket pkt;
  pkt.valid = false;
  pkt.hasSteps = false;

  portENTER_CRITICAL(&telemetryMux);
  pkt.m1Steps = sharedTelemetry.m1Steps;
  pkt.m2Steps = sharedTelemetry.m2Steps;
  pkt.rpm = sharedTelemetry.rpm;
  pkt.accel = sharedTelemetry.accel;
  pkt.servoExtRev = sharedTelemetry.servoExtRev;
  pkt.servoRetRev = sharedTelemetry.servoRetRev;
  pkt.servoMsRev = sharedTelemetry.servoMsRev;
  pkt.zOffsetRev = sharedTelemetry.zOffsetRev;
  memcpy(pkt.boxX, sharedTelemetry.boxX, sizeof(pkt.boxX));
  memcpy(pkt.boxZ, sharedTelemetry.boxZ, sizeof(pkt.boxZ));
  portEXIT_CRITICAL(&telemetryMux);

  if (Firebase.getJSON(fbData, "/asrs/config")) {
    FirebaseJson &cfgJson = fbData.jsonObject();
    FirebaseJsonData res;

    if (cfgJson.get(res, "m1_steps") && res.success) { pkt.m1Steps = res.intValue; pkt.hasSteps = true; pkt.valid = true; }
    if (cfgJson.get(res, "m2_steps") && res.success) { pkt.m2Steps = res.intValue; pkt.hasSteps = true; pkt.valid = true; }
    if (cfgJson.get(res, "rpm") && res.success && res.floatValue >= 1.0f) { pkt.rpm = res.floatValue; pkt.valid = true; }
    if (cfgJson.get(res, "accel") && res.success && res.floatValue >= 10.0f) { pkt.accel = res.floatValue; pkt.valid = true; }
    if (cfgJson.get(res, "servo_ext_rev") && res.success && res.floatValue > 0.0f) { pkt.servoExtRev = res.floatValue; pkt.valid = true; }
    if (cfgJson.get(res, "servo_ret_rev") && res.success && res.floatValue > 0.0f) { pkt.servoRetRev = res.floatValue; pkt.valid = true; }
    if (cfgJson.get(res, "servo_ms_rev") && res.success && res.intValue >= 100) { pkt.servoMsRev = (unsigned long)res.intValue; pkt.valid = true; }
    if (cfgJson.get(res, "z_offset") && res.success && res.floatValue != 0.0f) { pkt.zOffsetRev = res.floatValue; pkt.valid = true; }
    else if (cfgJson.get(res, "z_offset_rev") && res.success && res.floatValue != 0.0f) { pkt.zOffsetRev = res.floatValue; pkt.valid = true; }

    const char rows[3] = {'A', 'B', 'C'};
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        char pathX[32], pathZ[32];
        snprintf(pathX, sizeof(pathX), "coordinates/%c%d/x", rows[r], c + 1);
        snprintf(pathZ, sizeof(pathZ), "coordinates/%c%d/z", rows[r], c + 1);
        if (cfgJson.get(res, pathX) && res.success) { pkt.boxX[r][c] = res.floatValue; pkt.valid = true; }
        if (cfgJson.get(res, pathZ) && res.success) { pkt.boxZ[r][c] = res.floatValue; pkt.valid = true; }
      }
    }
  }

  if (!pkt.hasSteps) {
    if (Firebase.getJSON(fbData, "/asrs/telemetry")) {
      FirebaseJson &telJson = fbData.jsonObject();
      FirebaseJsonData res;
      if (telJson.get(res, "m1_steps") && res.success) { pkt.m1Steps = res.intValue; pkt.hasSteps = true; pkt.valid = true; }
      if (telJson.get(res, "m2_steps") && res.success) { pkt.m2Steps = res.intValue; pkt.hasSteps = true; pkt.valid = true; }
      if (telJson.get(res, "rpm") && res.success && res.floatValue >= 1.0f) { pkt.rpm = res.floatValue; pkt.valid = true; }
      if (telJson.get(res, "accel") && res.success && res.floatValue >= 10.0f) { pkt.accel = res.floatValue; pkt.valid = true; }
      if (telJson.get(res, "servo_ext_rev") && res.success && res.floatValue > 0.0f) { pkt.servoExtRev = res.floatValue; pkt.valid = true; }
      if (telJson.get(res, "servo_ret_rev") && res.success && res.floatValue > 0.0f) { pkt.servoRetRev = res.floatValue; pkt.valid = true; }
      if (telJson.get(res, "servo_ms_rev") && res.success && res.intValue >= 100) { pkt.servoMsRev = (unsigned long)res.intValue; pkt.valid = true; }
      if (telJson.get(res, "z_offset") && res.success && res.floatValue != 0.0f) { pkt.zOffsetRev = res.floatValue; pkt.valid = true; }
    }
  }

  if (pkt.valid) {
    xQueueOverwrite(restoreQueue, &pkt);
  }
  return pkt.valid;
}

void syncSnapshotToFirebase() {
  TelemetrySnapshot snap;
  bool doConfigSave = false;

  portENTER_CRITICAL(&telemetryMux);
  snap = sharedTelemetry;
  pendingFirebaseSync = false;
  if (pendingConfigSave) {
    doConfigSave = true;
    pendingConfigSave = false;
  }
  portEXIT_CRITICAL(&telemetryMux);

  FirebaseJson json;
  json.set("status", snap.statusMsg);
  json.set("locked", snap.locked);
  json.set("running", snap.running);
  json.set("m1_steps", (int)snap.m1Steps);
  json.set("m2_steps", (int)snap.m2Steps);
  json.set("m1_rev", snap.m1Rev);
  json.set("m2_rev", snap.m2Rev);
  json.set("servo_state", snap.servoState);
  json.set("servo_ext_rev", snap.servoExtRev);
  json.set("servo_ret_rev", snap.servoRetRev);
  json.set("servo_ms_rev", (int)snap.servoMsRev);
  json.set("box", snap.box);
  json.set("op", snap.op);
  json.set("step_stage", snap.stepStage);
  json.set("rpm", snap.rpm);
  json.set("accel", snap.accel);
  json.set("z_offset", snap.zOffsetRev);

  Firebase.updateNodeSilent(fbData, "/asrs/telemetry", json);

  if (doConfigSave) {
    saveConfigAndPositionToFirebase(snap);
  }
}

void firebaseTaskCode(void * pvParameters) {
  unsigned long lastPollTime = 0;
  unsigned long lastTelemetrySyncTime = 0;
  uint8_t consecutiveErrors = 0;

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (Firebase.ready()) {
      if (pendingConfigLoad) {
        pendingConfigLoad = false;
        if (!loadConfigAndPositionFromFirebase()) {
          portENTER_CRITICAL(&telemetryMux);
          pendingConfigSave = true;
          pendingFirebaseSync = true;
          portEXIT_CRITICAL(&telemetryMux);
        }
      }

      unsigned long now = millis();

      if (now - lastPollTime >= 350) {
        lastPollTime = now;

        if (Firebase.getString(fbData, "/asrs/control/command")) {
          consecutiveErrors = 0;
          String currentCommand = fbData.stringData();
          currentCommand.replace("\"", "");
          currentCommand.trim();

          if (currentCommand.length() > 0 && currentCommand != "NONE") {
            Firebase.setString(fbData, "/asrs/control/command", "NONE");

            CommandPacket pkt;
            strncpy(pkt.cmd, currentCommand.c_str(), sizeof(pkt.cmd) - 1);
            pkt.cmd[sizeof(pkt.cmd) - 1] = '\0';
            strcpy(pkt.source, "WEBSITE");

            if (xQueueSend(commandQueue, &pkt, 0) != pdTRUE) {
              CommandPacket dropped;
              xQueueReceive(commandQueue, &dropped, 0);
              xQueueSend(commandQueue, &pkt, 0);
            }
          }
        } else {
          consecutiveErrors++;
          if (consecutiveErrors >= 3) {
            consecutiveErrors = 0;
            fbData.pauseFirebase(true);
            vTaskDelay(pdMS_TO_TICKS(200));
            fbData.pauseFirebase(false);
          }
        }
      }

      if ((pendingFirebaseSync || pendingConfigSave) && (millis() - lastTelemetrySyncTime >= 400)) {
        lastTelemetrySyncTime = millis();
        syncSnapshotToFirebase();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  commandQueue = xQueueCreate(10, sizeof(CommandPacket));
  restoreQueue = xQueueCreate(1, sizeof(RestoredConfigPacket));

  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    esp_task_wdt_config_t twdt_config = {
      .timeout_ms = 30000,
      .idle_core_mask = 0,
      .trigger_panic = false
    };
    esp_task_wdt_reconfigure(&twdt_config);
  #else
    esp_task_wdt_init(30, false);
  #endif

  pinMode(INDICATOR_PIN_1, OUTPUT);
  pinMode(INDICATOR_PIN_2, OUTPUT);
  digitalWrite(INDICATOR_PIN_1, LOW);
  digitalWrite(INDICATOR_PIN_2, LOW);

  pinMode(GLOBAL_EN_PIN, OUTPUT);
  digitalWrite(GLOBAL_EN_PIN, LOW); 

  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(SERVO_PIN, SERVO_PWM_FREQ, SERVO_PWM_RES_BITS);
  #else
    ledcSetup(SERVO_LEDC_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RES_BITS);
    ledcAttachPin(SERVO_PIN, SERVO_LEDC_CHANNEL);
  #endif

  loadStateFromNVS();
  recalculateServoTimings();
  setServo360("STOP");

  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);

  if (microstepSetting == 8) {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, LOW);
  } else {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, LOW);
  }

  pinMode(LS_X_LEFT, INPUT_PULLUP);
  pinMode(LS_X_RIGHT, INPUT_PULLUP);
  pinMode(LS_Z_TOP, INPUT_PULLUP);
  pinMode(LS_Z_BOTTOM, INPUT_PULLUP);

  stepper1.enableOutputs();
  stepper2.enableOutputs();

  stepper1.setMinPulseWidth(3);
  stepper2.setMinPulseWidth(3);

  applySpeedAndRampSettings();
  requestFirebaseSync("SYSTEM ONLINE (RESTORING CONFIG...)", false);

  Serial.print("Menghubungkan ke WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi Terhubung!");

  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  fbData.setBSSLBufferSize(4096, 1024);
  fbData.setResponseSize(1024);

  config.timeout.socketConnection = 8000;
  config.timeout.sslHandshake = 12000;
  config.timeout.serverResponse = 8000;
  config.timeout.rtdbKeepAlive = 45000;
  config.timeout.rtdbStreamReconnect = 2000;
  config.timeout.rtdbStreamError = 5000;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  xTaskCreatePinnedToCore(firebaseTaskCode, "FirebaseTask", 16384, NULL, 1, NULL, 0);

  printSerialHelp();
}

// ==========================================
// TUGAS CORE 1: EKSEKUSI PULSA MOTOR, SERVO & STATE MACHINE ASRS
// ==========================================
void loop() {
  checkSerialAndQueueInput();

  if (isManualServoTimed) {
    if (millis() - manualServoStartMs >= manualServoTargetMs) {
      isManualServoTimed = false;
      setServo360("STOP");
      Serial.println(">> [SERVO 360 TIMED] Target putaran Servo tercapai! Servo STOP.");
      requestFirebaseSync("SERVO TARGET REV REACHED (STOP)");
    }
  }

  if (isLimitSwitchHit() && isMotorRunning) {
    Serial.println("\n[BAHAYA] LIMIT SWITCH TERTABRAK!");
    triggerEmergencyStop();
    requestFirebaseSync("EMERGENCY: LIMIT SWITCH HIT", true);
  }

  if (isMotorRunning && !isSystemLocked) {
    if (currentMode == MODE_CONTINUOUS) {
      if (isHomingLimitSwitchPressed()) {
        stopMotorsNormal();
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);
        Serial.println("\n>> [HOMING SELESAI] Limit Switch Pin 19 tertekan! Posisi di-set ke (0,0) & Disimpan Online.");
        requestFirebaseSync("HOMING COMPLETE (AT 0,0)", true);
      } else {
        stepper1.runSpeed();
        stepper2.runSpeed();
      }
    } 
    else if (currentMode == MODE_POSITION) {
      bool m1Moving = (stepper1.distanceToGo() != 0);
      bool m2Moving = (stepper2.distanceToGo() != 0);

      if (m1Moving) stepper1.run();
      if (m2Moving) stepper2.run();

      if (!m1Moving && !m2Moving) {
        isMotorRunning = false;
        currentMode = MODE_IDLE;
        digitalWrite(INDICATOR_PIN_1, LOW);
        digitalWrite(INDICATOR_PIN_2, LOW);
        bool atHome = (stepper1.currentPosition() == 0 && stepper2.currentPosition() == 0);
        Serial.println(">> [SELESAI] Target putaran tercapai! Posisi disimpan online.");
        requestFirebaseSync(atHome ? "HOME REACHED (0,0)" : "TARGET REACHED (IDLE)", true);
      }
    }
    else if (currentMode == MODE_ASRS_SEQUENCE) {
      switch (asrsState) {
        case ASRS_STEP_3_MOVE_X:
          if (stepper1.distanceToGo() != 0) {
            stepper1.run();
          } else {
            long targetZSteps = round(targetZRev * stepsPerRevolution * microstepSetting);
            stepper2.moveTo(targetZSteps);
            asrsState = ASRS_STEP_4_MOVE_Z;
            Serial.printf("4. [PROC] X Sampai! Menggerakkan Z-Axis ke %.2f Rev (%ld steps)...\n", targetZRev, targetZSteps);
            requestFirebaseSync("STEP 4: MOVING Z -> " + String(activeBoxAddress), true);
          }
          break;

        case ASRS_STEP_4_MOVE_Z:
          if (stepper2.distanceToGo() != 0) {
            stepper2.run();
          } else {
            recalculateServoTimings();
            isManualServoTimed = false;
            setServo360("EXTEND");
            servoTimerStart = millis();
            asrsState = ASRS_STEP_5A_SERVO_EXTEND;
            Serial.printf("5a. [ACTUATE] Z Sampai! SERVO EXTEND selama %lu ms (%.2f Rev)...\n",
                          SERVO_EXTEND_MS, servoExtractRev);
            requestFirebaseSync("STEP 5A: SERVO EXTENDING", true);
          }
          break;

        case ASRS_STEP_5A_SERVO_EXTEND:
          if (millis() - servoTimerStart >= SERVO_EXTEND_MS) {
            setServo360("STOP");
            servoTimerStart = millis();
            asrsState = ASRS_STEP_5A_SETTLE;
          }
          break;

        case ASRS_STEP_5A_SETTLE:
          if (millis() - servoTimerStart >= 150) {
            bool isRetrieval = (strcmp(activeOperation, "RETRIEVAL") == 0);
            float absOff = fabs(zLiftOffsetRev);
            float offsetRev = isRetrieval ? absOff : -absOff;
            long offsetSteps = round(offsetRev * stepsPerRevolution * microstepSetting);
            stepper2.move(offsetSteps);
            asrsState = ASRS_STEP_5B_Z_LIFT_DROP;
            Serial.printf("5b. [ACTUATE] Menggerakkan Motor Z sejauh %.2f Rev (%ld steps)...\n",
                          offsetRev, offsetSteps);
            requestFirebaseSync(isRetrieval ? "STEP 5B: Z LIFT BOX" : "STEP 5B: Z DROP BOX");
          }
          break;

        case ASRS_STEP_5B_Z_LIFT_DROP:
          if (stepper2.distanceToGo() != 0) {
            stepper2.run();
          } else {
            recalculateServoTimings();
            setServo360("RETRACT");
            servoTimerStart = millis();
            asrsState = ASRS_STEP_5C_SERVO_RETRACT;
            Serial.printf("5c. [ACTUATE] Gerak Z Selesai! SERVO RETRACT selama %lu ms (%.2f Rev)...\n",
                          SERVO_RETRACT_MS, servoRetractRev);
            requestFirebaseSync("STEP 5C: SERVO RETRACTING", true);
          }
          break;

        case ASRS_STEP_5C_SERVO_RETRACT:
          if (millis() - servoTimerStart >= SERVO_RETRACT_MS) {
            setServo360("STOP");
            servoTimerStart = millis();
            asrsState = ASRS_STEP_5C_SETTLE;
          }
          break;

        case ASRS_STEP_5C_SETTLE:
          if (millis() - servoTimerStart >= 150) {
            stepper1.moveTo(0);
            stepper2.moveTo(0);
            asrsState = ASRS_STEP_6_BACK_HOME;
            Serial.println("6. [HOME] Mengembalikan Motor X & Z ke Home (0, 0)...");
            requestFirebaseSync("STEP 6: RETURNING TO HOME (0,0)");
          }
          break;

        case ASRS_STEP_6_BACK_HOME: {
          bool m1Home = (stepper1.distanceToGo() != 0);
          bool m2Home = (stepper2.distanceToGo() != 0);

          if (m1Home) stepper1.run();
          if (m2Home) stepper2.run();

          if (!m1Home && !m2Home) {
            isMotorRunning = false;
            currentMode = MODE_IDLE;
            asrsState = ASRS_NONE;
            digitalWrite(INDICATOR_PIN_1, LOW);
            digitalWrite(INDICATOR_PIN_2, LOW);
            Serial.println(">> [SIKLUS SELESAI] ASRS telah kembali ke Home (0, 0) & posisi disimpan online!");
            requestFirebaseSync("CYCLE COMPLETE - AT HOME (0,0)", true);
          }
          break;
        }

        default:
          break;
      }
    }
  }
}
