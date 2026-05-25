#include <Bluepad32.h>
#include <Encoder.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUDP.h>

// ---------- Pin map ----------
constexpr int L_RPWM = 25;
constexpr int L_LPWM = 26;
// L_REN/L_LEN are hardwired to 5V
constexpr int ENC_L_A = 18;
constexpr int ENC_L_B = 19;

constexpr int R_RPWM = 4;
constexpr int R_LPWM = 13;
// R_REN/R_LEN are hardwired to 5V
constexpr int ENC_R_A = 21;
constexpr int ENC_R_B = 22;

constexpr int BLUE_LED = 2;

// -------- Encoders & PID ---------
Encoder* encLeft  = nullptr;
Encoder* encRight = nullptr;

const float MAX_RPM = 110.0f;
const float PPR     = 11.0f * 90.0f;
const float LOOP_MS = 50.0f;
unsigned long lastLoop = 0;

float kp = 1.0 / MAX_RPM, ki = 0.0001f, kd = 0.005f;
static float integralLeft  = 0.0f, integralRight  = 0.0f;
static float prevErrorLeft = 0.0f, prevErrorRight = 0.0f;
float targetLeft  = 0.0f;
float targetRight = 0.0f;

// ---------- Mode & WiFi ----------
bool   autoMode  = false;
String autoCmd   = "STOP";

unsigned long lastToggleMs = 0;

const char* ssid     = "MEHDI 6409";
const char* password = "#8567D2b";
WiFiUDP udp;
const int UDP_PORT = 4210;

// ---------- Gamepad & Timers ----------
GamepadPtr    myGamepad  = nullptr;
uint32_t      lastCmdMs  = 0;
constexpr uint32_t CMD_TIMEOUT_MS = 400;

unsigned long lastPrintMs = 0;
const unsigned long PRINT_MS = 400;

// ============================================================
//  MOTOR CLASS  — kick only fires for real commands,
//  fixing the creep-at-zero bug without needing /100 hack
// ============================================================
class MotorBTS7960 {
public:
  MotorBTS7960(int rpwm, int lpwm, int chR, int chL)
    : rpwm_(rpwm), lpwm_(lpwm), chR_(chR), chL_(chL) {}

  void begin() {
    ledcSetup(chR_, 150, 10); ledcSetup(chL_, 150, 10);
    ledcAttachPin(rpwm_, chR_); ledcAttachPin(lpwm_, chL_);
  }

  void set(float speed) {
    if (speed >  1.0f) speed =  1.0f;
    if (speed < -1.0f) speed = -1.0f;
    float    mag  = fabs(speed);
    uint32_t duty = (uint32_t)(mag * 1023);

    if (duty == 0) { stop(); return; }

    // Kick only applies when there's a real intentional command,
    // not when a tiny PID correction sneaks through
    if (mag > 0.15f && duty < 200) duty = 200;

    if (speed >= 0) { ledcWrite(chL_, 0);    ledcWrite(chR_, duty); }
    else            { ledcWrite(chR_, 0);    ledcWrite(chL_, duty); }
  }

  void stop() { ledcWrite(chR_, 0); ledcWrite(chL_, 0); }

private:
  int rpwm_, lpwm_, chR_, chL_;
};

MotorBTS7960 leftMotor (L_RPWM, L_LPWM, 0, 1);
MotorBTS7960 rightMotor(R_RPWM, R_LPWM, 2, 3);

// ============================================================
//  AUTO COMMAND HANDLER
//  Handles all 10 commands sent by the pilot script
// ============================================================
void executeAutoCmd(const String& cmd) {
  const float FWD   = 0.55f;
  const float SLOW  = 0.30f;  // Cautious speed near crops
  const float TURN  = 0.40f;  // Gentle drift correction
  const float HTURN = 0.65f;  // Large path deviation

  if      (cmd == "FORWARD")     { targetLeft =  FWD;          targetRight =  FWD;          }
  else if (cmd == "BACK")        { targetLeft = -FWD;          targetRight = -FWD;          }

  // Steering — normal
  else if (cmd == "LEFT")        { targetLeft = -TURN;         targetRight =  TURN;         }
  else if (cmd == "RIGHT")       { targetLeft =  TURN;         targetRight = -TURN;         }

  // Steering — hard (significant path deviation)
  else if (cmd == "HARD_LEFT")   { targetLeft = -HTURN;        targetRight =  HTURN;        }
  else if (cmd == "HARD_RIGHT")  { targetLeft =  HTURN;        targetRight = -HTURN;        }

  // Slow cruise — plant/crop detected ahead
  else if (cmd == "SLOW")        { targetLeft =  SLOW;         targetRight =  SLOW;         }
  else if (cmd == "SLOW_LEFT")   { targetLeft = -SLOW;         targetRight =  SLOW * 1.2f;  }
  else if (cmd == "SLOW_RIGHT")  { targetLeft =  SLOW * 1.2f; targetRight = -SLOW;         }

  // Path lost — slow spin to re-acquire dirt
  else if (cmd == "SEARCH")      { targetLeft =  0.25f;        targetRight = -0.25f;        }

  else if (cmd == "STOP")        { targetLeft =  0.0f;         targetRight =  0.0f;         }
}

// ---------- Bluepad32 callbacks ----------
void onConnectedGamepad(GamepadPtr gp) {
  myGamepad = gp;
  Serial.println("Gamepad connected");
}
void onDisconnectedGamepad(GamepadPtr gp) {
  if (myGamepad == gp) myGamepad = nullptr;
  Serial.println("Gamepad disconnected");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(BLUE_LED, OUTPUT);

  leftMotor.begin();
  rightMotor.begin();

  encLeft  = new Encoder(ENC_L_A, ENC_L_B);
  encRight = new Encoder(ENC_R_A, ENC_R_B);

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500); Serial.print("."); tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi OK! IP: "); Serial.println(WiFi.localIP());
    udp.begin(UDP_PORT);
  } else {
    Serial.println("\nWiFi FAILED — manual mode only.");
  }

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  BP32.update();

  // ----------------------------------------------------------
  //  1. UDP RECEIVER
  // ----------------------------------------------------------
  int pktLen = udp.parsePacket();
  if (pktLen > 0) {
    char buf[32];
    udp.read(buf, sizeof(buf) - 1);
    buf[min(pktLen, (int)sizeof(buf) - 1)] = '\0';  // safe null-term

    autoCmd = String(buf);
    autoCmd.trim();

    // Safety timer resets on every packet, regardless of mode
    lastCmdMs = millis();
  }

  // ----------------------------------------------------------
  //  2. MODE TOGGLE & MANUAL STICK INPUT
  // ----------------------------------------------------------
  if (myGamepad && myGamepad->isConnected()) {

    // Y button (0x0008) toggles auto/manual with 1s debounce
    if ((myGamepad->buttons() & 0x0008) && (millis() - lastToggleMs > 1000)) {
      autoMode = !autoMode;
      lastToggleMs = millis();
      leftMotor.stop(); rightMotor.stop();
      integralLeft = 0; integralRight = 0;
      prevErrorLeft = 0; prevErrorRight = 0;
      Serial.println(autoMode ? ">>> AUTO MODE" : ">>> MANUAL MODE");
    }

    if (!autoMode) {
      targetLeft  = (float)(-myGamepad->axisY())  / 512.0f * 0.8f;
      targetRight = (float)(-myGamepad->axisRY()) / 512.0f * 0.8f;
      lastCmdMs   = millis();
    }
  }

  // Apply auto commands when in auto mode
  if (autoMode) executeAutoCmd(autoCmd);

  // ----------------------------------------------------------
  //  3. PID LOOP  (runs every LOOP_MS for both modes)
  // ----------------------------------------------------------
  if (millis() - lastLoop > LOOP_MS) {
    float deltaT = (millis() - lastLoop) / 1000.0f;
    lastLoop = millis();

    // --- Left Motor ---
    long  dL   = encLeft->read() / 4;
    float actL = (dL / deltaT) * (60.0f / PPR);
    encLeft->write(0);

    if (fabs(targetLeft) < 0.15f) {
      targetLeft   = 0;
      integralLeft = 0;
      prevErrorLeft = 0;
      leftMotor.stop();
    } else {
      float errL      = targetLeft * MAX_RPM - actL;
      integralLeft    = constrain(integralLeft + errL * deltaT, -0.5f, 0.5f);
      float pTermL    = errL * kp;
      float iTermL    = integralLeft * ki;
      float dTermL    = (errL - prevErrorLeft) / deltaT * kd;
      prevErrorLeft   = errL;
      float corrL     = constrain(pTermL + iTermL + dTermL, -1.0f, 1.0f);
      // Full PID correction — kick fix in set() handles the creep issue
      leftMotor.set(constrain(targetLeft + corrL, -1.0f, 1.0f));
    }

    // --- Right Motor ---
    long  dR   = encRight->read() / 4;
    float actR = (dR / deltaT) * (60.0f / PPR);
    encRight->write(0);

    if (fabs(targetRight) < 0.15f) {
      targetRight   = 0;
      integralRight = 0;
      prevErrorRight = 0;
      rightMotor.stop();
    } else {
      float errR      = targetRight * MAX_RPM - actR;
      integralRight   = constrain(integralRight + errR * deltaT, -0.5f, 0.5f);
      float pTermR    = errR * kp;
      float iTermR    = integralRight * ki;
      float dTermR    = (errR - prevErrorRight) / deltaT * kd;
      prevErrorRight  = errR;
      float corrR     = constrain(pTermR + iTermR + dTermR, -1.0f, 1.0f);
      rightMotor.set(constrain(targetRight + corrR, -1.0f, 1.0f));
    }

    // Serial telemetry
    if (millis() - lastPrintMs >= PRINT_MS) {
      Serial.printf("L: tgt=%.1f act=%.1f | R: tgt=%.1f act=%.1f | Mode: %s | CMD: %s\n",
                    targetLeft  * MAX_RPM, actL,
                    targetRight * MAX_RPM, actR,
                    autoMode ? "AUTO" : "MAN",
                    autoCmd.c_str());
      lastPrintMs = millis();
    }
  }

  // ----------------------------------------------------------
  //  4. LED STATUS
  //  OFF        = Auto mode active
  //  SOLID      = Manual + gamepad connected
  //  DOUBLE BLINK = WiFi only (no gamepad)
  //  RAPID BLINK  = No connection at all
  // ----------------------------------------------------------
  static unsigned long lastLEDStep = 0;
  if (millis() - lastLEDStep > 100) {
    lastLEDStep = millis();
    bool bt   = (myGamepad && myGamepad->isConnected());
    bool wifi = (WiFi.status() == WL_CONNECTED);

    if (autoMode)       digitalWrite(BLUE_LED, LOW);
    else if (bt)        digitalWrite(BLUE_LED, HIGH);
    else if (wifi) {
      int b = (millis() / 200) % 10;
      digitalWrite(BLUE_LED, (b == 0 || b == 2) ? HIGH : LOW);
    }
    else                digitalWrite(BLUE_LED, !digitalRead(BLUE_LED));
  }

  // ----------------------------------------------------------
  //  5. SAFETY TIMEOUT — stop if no command received recently
  // ----------------------------------------------------------
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    targetLeft    = 0.0f;  targetRight    = 0.0f;
    integralLeft  = 0.0f;  integralRight  = 0.0f;
    prevErrorLeft = 0.0f;  prevErrorRight = 0.0f;
    leftMotor.stop();
    rightMotor.stop();
  }
}
