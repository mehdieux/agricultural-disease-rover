#include <Bluepad32.h>
#include <Encoder.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUDP.h>

// ---------- Pin map ----------
constexpr int L_RPWM = 25;
constexpr int L_LPWM = 26;
// L_REN/L_LEN are hardwired to 5V
constexpr int ENC_L_A = 18; //green
constexpr int ENC_L_B = 19; //yellow

constexpr int R_RPWM = 4;  // One direction only (MOSFET gate)
constexpr int R_LPWM = 27; // unused: right side is forward-only
// R_REN/R_LEN are hardwired to 5V
constexpr int ENC_R_A = 21;//green
constexpr int ENC_R_B = 22;//yellow

constexpr int BLUE_LED = 2;

// ---------- Direction config ----------
// The right (MOSFET) motor can only spin one way, and that way is physically
// "backwards" vs. the original wiring. Instead of rewiring it, the left motor
// is reversed in code so both wheels match.
constexpr float LEFT_MOTOR_DIR = -1.0f;  // reverses left motor output
constexpr float LEFT_ENC_SIGN  = -1.0f;  // left encoder flips with the motor
constexpr float RIGHT_ENC_SIGN = -1.0f;  // right motor spins "backwards" vs. its original wiring
// If "act" shows negative while the wheel spins forward, flip that sign to +1.0f

// ---------- PWM config ----------
constexpr int PWM_FREQ = 2000;
constexpr int PWM_RES  = 10;

// -------- Encoders & PID ---------
Encoder* encLeft  = nullptr;
Encoder* encRight = nullptr;

const float MAX_RPM = 110.0f;
const float PPR     = 11.0f * 90.0f;
const float LOOP_MS = 50.0f;
unsigned long lastLoop = 0;

float kp = 1.0f / MAX_RPM;
float ki = 0.0001f;
float kd = 0.0f;

static float integralLeft  = 0.0f;
static float integralRight = 0.0f;
static float prevErrorLeft = 0.0f;
static float prevErrorRight = 0.0f;

// "commanded" = raw desired target straight from trigger/auto-cmd (can jump instantly)
// "target" = ramped value actually fed to the PID (moves toward commanded at a limited rate)
float commandedLeft  = 0.0f;
float commandedRight = 0.0f;

float targetLeft  = 0.0f;
float targetRight = 0.0f;

// Shared acceleration ramp, in RPM/second. Both wheels get the same ramp so they
// accelerate together; the independent PI loops compensate for the different hardware.
constexpr float MAX_ACCEL_RPM_S = 150.0f;

// -------- Debug / measured outputs --------
float outputLeft  = 0.0f;
float outputRight = 0.0f;

float pTermLeft = 0.0f;
float iTermLeft = 0.0f;
float dTermLeft = 0.0f;

float pTermRight = 0.0f;
float iTermRight = 0.0f;
float dTermRight = 0.0f;

// raw encoder deltas, kept around just for the debug print
long dL_debug = 0;
long dR_debug = 0;

// Right encoder stale-reading guard. If the measured speed stays near zero
// for this long while a much higher speed is commanded, the output is frozen
// instead of being driven to max PWM against a possibly-dead encoder signal.
constexpr unsigned long STALE_LIMIT_MS = 7000;
int staleCounterRight = 0;

// Breakaway PWM per side, measured with the calibration sketch (unloaded):
// left starts at ~120, right starts at ~400. Padded ~10% for load.
// Re-measure after the rover is assembled.
constexpr uint32_t MIN_PWM_LEFT  = 130;
constexpr uint32_t MIN_PWM_RIGHT = 410;

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
//  MOTOR CLASS
// ============================================================
class MotorBTS7960 {
public:
  MotorBTS7960(int rpwm, int lpwm, int chR, int chL)
    : rpwm_(rpwm), lpwm_(lpwm), chR_(chR), chL_(chL) {}

  void begin() {
    ledcSetup(chR_, PWM_FREQ, PWM_RES);
    ledcSetup(chL_, PWM_FREQ, PWM_RES);

    ledcAttachPin(rpwm_, chR_);
    ledcAttachPin(lpwm_, chL_);

    lastDuty_ = 0;
  }

  // minDuty = this motor's breakaway PWM. The speed range is scaled so that
  // "just above zero" maps to minDuty and "full" stays at 1023, so both motors
  // start moving at the same low commanded speed instead of being clamped.
  void set(float speed, uint32_t minDuty) {
    if (speed > 1.0f)  speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    float mag = fabs(speed);

    if (mag < 0.02f) {
      stop();
      return;
    }

    uint32_t duty = minDuty + (uint32_t)(mag * (1023 - minDuty));

    lastDuty_ = duty;

    if (speed >= 0) {
      ledcWrite(chL_, 0);
      ledcWrite(chR_, duty);
    }
    else {
      ledcWrite(chR_, 0);
      ledcWrite(chL_, duty);
    }
  }

  void stop() {
    ledcWrite(chR_, 0);
    ledcWrite(chL_, 0);
    lastDuty_ = 0;
  }

  uint32_t getLastDuty() const {
    return lastDuty_;
  }

private:
  int rpwm_;
  int lpwm_;
  int chR_;
  int chL_;

  uint32_t lastDuty_ = 0;
};

// channels 4/5 for left, 6/7 for right
MotorBTS7960 leftMotor (L_RPWM, L_LPWM, 4, 5);
MotorBTS7960 rightMotor(R_RPWM, R_LPWM, 6, 7);

// ============================================================
//  AUTO COMMAND HANDLER
// ============================================================
// l / r are the DRIVER's view (left wheel / right wheel as seen from the
// new front). Because the old rear is now the front, the sides are swapped
// when written into commandedLeft/commandedRight (the motors in code).
// Unknown commands leave the targets unchanged.
void executeAutoCmd(const String& cmd) {
  const float FWD   = 0.55f;
  const float SLOW  = 0.30f;
  const float TURN  = 0.40f;
  const float HTURN = 0.65f;

  float l, r;

  if      (cmd == "FORWARD")     { l =  FWD;         r =  FWD;         }
  else if (cmd == "BACK")        { l = -FWD;         r = -FWD;         }
  else if (cmd == "LEFT")        { l = -TURN;        r =  TURN;        }
  else if (cmd == "RIGHT")       { l =  TURN;        r = -TURN;        }
  else if (cmd == "HARD_LEFT")   { l = -HTURN;       r =  HTURN;       }
  else if (cmd == "HARD_RIGHT")  { l =  HTURN;       r = -HTURN;       }
  else if (cmd == "SLOW")        { l =  SLOW;        r =  SLOW;        }
  else if (cmd == "SLOW_LEFT")   { l = -SLOW;        r =  SLOW * 1.2f; }
  else if (cmd == "SLOW_RIGHT")  { l =  SLOW * 1.2f; r = -SLOW;        }
  else if (cmd == "SEARCH")      { l =  0.25f;       r = -0.25f;       }
  else if (cmd == "STOP")        { l =  0.0f;        r =  0.0f;        }
  else return;  // unknown command: leave targets unchanged

  // Front end is now the old rear, so the sides are swapped
  commandedLeft  = r;
  commandedRight = l;
}

// ---------- Bluepad32 callbacks ----------
void onConnectedGamepad(GamepadPtr gp) {
  myGamepad = gp;
  Serial.println("Gamepad connected");
}

void onDisconnectedGamepad(GamepadPtr gp) {
  if (myGamepad == gp)
    myGamepad = nullptr;

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

  // Start PID timing from setup instead of millis() == 0
  lastLoop = millis();

  WiFi.begin(ssid, password);

  Serial.print("Connecting WiFi");

  int tries = 0;

  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi OK! IP: ");
    Serial.println(WiFi.localIP());

    udp.begin(UDP_PORT);
  }
  else {
    Serial.println("\nWiFi FAILED — manual mode only.");
  }

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // Prevent safety timeout from firing immediately at boot
  lastCmdMs = millis();
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
    buf[min(pktLen, (int)sizeof(buf) - 1)] = '\0';

    autoCmd = String(buf);
    autoCmd.trim();

    lastCmdMs = millis();
  }

  // ----------------------------------------------------------
  //  2. MODE TOGGLE & MANUAL TRIGGER/STICK INPUT
  // ----------------------------------------------------------
  if (myGamepad && myGamepad->isConnected()) {

    if ((myGamepad->buttons() & 0x0008) &&
        (millis() - lastToggleMs > 1000)) {

      autoMode = !autoMode;
      lastToggleMs = millis();

      leftMotor.stop();
      rightMotor.stop();

      integralLeft = 0;
      integralRight = 0;

      prevErrorLeft = 0;
      prevErrorRight = 0;

      // also zero the ramp state on mode toggle so we don't
      // resume from a stale target/commanded value
      commandedLeft = 0;
      commandedRight = 0;
      targetLeft = 0;
      targetRight = 0;

      Serial.println(autoMode ? ">>> AUTO MODE" : ">>> MANUAL MODE");
    }

    if (!autoMode) {

      float throttleNorm =
          (float)myGamepad->throttle() / 1023.0f;

      float brakeNorm =
          (float)myGamepad->brake() / 1023.0f;

      float base = throttleNorm - brakeNorm;

      float steer =
          (float)myGamepad->axisX() / 512.0f;

      if (fabs(steer) < 0.08f)
        steer = 0.0f;

      const float STEER_GAIN = 0.6f;

      float driverLeft  = base + steer * STEER_GAIN;
      float driverRight = base - steer * STEER_GAIN;

      // Sides swapped: the driver's right wheel is the left motor in code.
      // Right motor stays forward-only, so it's clamped to [0, 1].
      // The ramp (in the PID loop below) turns these into targetLeft/targetRight.
      commandedLeft  = constrain(driverRight, -1.0f, 1.0f) * 0.8f;
      commandedRight = constrain(driverLeft,   0.0f, 1.0f) * 0.8f;

      lastCmdMs = millis();
    }
  }

  if (autoMode)
    executeAutoCmd(autoCmd);

  // ----------------------------------------------------------
  //  3. PID LOOP
  // ----------------------------------------------------------
  if (millis() - lastLoop > LOOP_MS) {

    unsigned long now = millis();

    float deltaT =
        (now - lastLoop) / 1000.0f;

    lastLoop = now;

    // Apply the deadzone to the *commanded* value, then ramp
    // targetLeft/targetRight toward it at a limited rate. Both wheels
    // share the same ramp so they accelerate together; the PID loops
    // below independently compensate for the different hardware.
    float cmdL = (fabs(commandedLeft)  < 0.15f) ? 0.0f : commandedLeft;
    float cmdR = (fabs(commandedRight) < 0.15f) ? 0.0f : commandedRight;

    float maxStep =
        (MAX_ACCEL_RPM_S / MAX_RPM) * deltaT;

    targetLeft +=
        constrain(cmdL - targetLeft, -maxStep, maxStep);

    targetRight +=
        constrain(cmdR - targetRight, -maxStep, maxStep);

    // ========================================================
    // LEFT MOTOR
    // ========================================================
    long dL = encLeft->read();
    dL_debug = dL;

    // LEFT_ENC_SIGN: the left motor is reversed, so its encoder count flips too
    float actL =
        LEFT_ENC_SIGN *
        (dL / deltaT) *
        (60.0f / (PPR * 4.0f));

    encLeft->write(0);

    // Only fully stop once BOTH the commanded value is in the
    // deadzone AND the ramped target has actually settled near zero.
    // This lets a stop command ramp down smoothly instead of snapping.
    if (cmdL == 0.0f && fabs(targetLeft) < 0.02f) {

      targetLeft = 0;

      integralLeft = 0;
      prevErrorLeft = 0;

      pTermLeft = 0;
      iTermLeft = 0;
      dTermLeft = 0;

      outputLeft = 0;

      leftMotor.stop();

    }
    else {

      float errL =
          targetLeft * MAX_RPM - actL;

      integralLeft =
          constrain(
              integralLeft + errL * deltaT,
              -0.5f,
              0.5f
          );

      pTermLeft =
          errL * kp;

      iTermLeft =
          integralLeft * ki;

      dTermLeft =
          (errL - prevErrorLeft) /
          deltaT * kd;

      prevErrorLeft = errL;

      float corrL =
          constrain(
              pTermLeft +
              iTermLeft +
              dTermLeft,
              -1.0f,
              1.0f
          );

      // Left is bidirectional, but the output is sign-locked to the
      // direction of targetLeft. Driving forward (targetLeft > 0) clamps the
      // output to [0, 1], so the PID can never push it negative and
      // reverse/brake mid-drive (this was the original stuttering bug).
      // Reverse (targetLeft < 0) clamps to [-1, 0] the same way, mirrored.
      float loLeft, hiLeft;

      if (targetLeft > 0.0f) {
        loLeft = 0.0f;
        hiLeft = 1.0f;
      }
      else if (targetLeft < 0.0f) {
        loLeft = -1.0f;
        hiLeft = 0.0f;
      }
      else {
        loLeft = 0.0f;
        hiLeft = 0.0f;
      }

      outputLeft =
          constrain(
              targetLeft + corrL,
              loLeft,
              hiLeft
          );

      // LEFT_MOTOR_DIR flips the physical direction; all the logic above
      // stays in "logical forward" terms.
      leftMotor.set(outputLeft * LEFT_MOTOR_DIR, MIN_PWM_LEFT);
    }

    // ========================================================
    // RIGHT MOTOR
    // ========================================================
    // Right stays forward-only (hardware constraint).
    long dR = encRight->read();
    dR_debug = dR;

    // RIGHT_ENC_SIGN: the MOSFET motor spins opposite to its original wiring
    float actR =
        RIGHT_ENC_SIGN *
        (dR / deltaT) *
        (60.0f / (PPR * 4.0f));

    encRight->write(0);

    if (cmdR == 0.0f && fabs(targetRight) < 0.02f) {

      targetRight = 0;

      integralRight = 0;
      prevErrorRight = 0;

      pTermRight = 0;
      iTermRight = 0;
      dTermRight = 0;

      outputRight = 0;

      staleCounterRight = 0; // reset stale guard on a real stop

      rightMotor.stop();

    }
    else {

      // Stale-reading guard. The right encoder sometimes reports small
      // or negative RPM (noise/glitch) even while genuinely stalled or
      // signal-dropped, so "measured speed <= 5 RPM while 20+ RPM is
      // commanded" is treated as suspect.
      bool suspectReading =
          (targetRight * MAX_RPM >= 20.0f) && (actR <= 5.0f);

      if (suspectReading) {
        staleCounterRight += (int)LOOP_MS;
      } else {
        staleCounterRight = 0;
      }

      bool staleRight = staleCounterRight >= (int)STALE_LIMIT_MS;

      if (staleRight) {
        // Freeze: keep whatever outputRight/PWM was already set, don't
        // recompute the correction, don't let it grow further.
        rightMotor.set(outputRight, MIN_PWM_RIGHT);
      }
      else {

        float errR =
            targetRight * MAX_RPM - actR;

        integralRight =
            constrain(
                integralRight + errR * deltaT,
                -0.5f,
                0.5f
            );

        pTermRight =
            errR * kp;

        iTermRight =
            integralRight * ki;

        dTermRight =
            (errR - prevErrorRight) /
            deltaT * kd;

        prevErrorRight = errR;

        float corrR =
            constrain(
                pTermRight +
                iTermRight +
                dTermRight,
                -1.0f,
                1.0f
            );

        outputRight =
            constrain(
                targetRight + corrR,
                0.0f,
                1.0f
            );

        rightMotor.set(outputRight, MIN_PWM_RIGHT);
      }
    }

    // --------------------------------------------------------
    // SERIAL DEBUG
    // --------------------------------------------------------
    if (millis() - lastPrintMs >= PRINT_MS) {

      Serial.printf(
        "L: tgt=%6.1f act=%6.1f dL=%5ld out=%6.3f PWM=%4lu | "
        "R: tgt=%6.1f act=%6.1f dR=%5ld out=%6.3f PWM=%4lu\n",

        targetLeft * MAX_RPM,
        actL,
        dL_debug,
        outputLeft,
        leftMotor.getLastDuty(),

        targetRight * MAX_RPM,
        actR,
        dR_debug,
        outputRight,
        rightMotor.getLastDuty()
      );

      Serial.printf(
        "    L PID: P=%+.3f I=%+.3f D=%+.3f | "
        "R PID: P=%+.3f I=%+.3f D=%+.3f\n",

        pTermLeft,
        iTermLeft,
        dTermLeft,

        pTermRight,
        iTermRight,
        dTermRight
      );

      lastPrintMs = millis();
    }
  }

  // ----------------------------------------------------------
  //  4. LED STATUS
  // ----------------------------------------------------------
  static unsigned long lastLEDStep = 0;

  if (millis() - lastLEDStep > 100) {

    lastLEDStep = millis();

    bool bt =
        (myGamepad && myGamepad->isConnected());

    bool wifi =
        (WiFi.status() == WL_CONNECTED);

    if (autoMode)
      digitalWrite(BLUE_LED, LOW);

    else if (bt)
      digitalWrite(BLUE_LED, HIGH);

    else if (wifi) {

      int b = (millis() / 200) % 10;

      digitalWrite(
          BLUE_LED,
          (b == 0 || b == 2) ? HIGH : LOW
      );
    }

    else {
      digitalWrite(
          BLUE_LED,
          !digitalRead(BLUE_LED)
      );
    }
  }

  // ----------------------------------------------------------
  //  5. SAFETY TIMEOUT
  // ----------------------------------------------------------
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {

    // also zero the commanded values so the ramp doesn't try to
    // resume toward a stale target once commands come back
    commandedLeft  = 0.0f;
    commandedRight = 0.0f;

    targetLeft  = 0.0f;
    targetRight = 0.0f;

    integralLeft  = 0.0f;
    integralRight = 0.0f;

    prevErrorLeft  = 0.0f;
    prevErrorRight = 0.0f;

    outputLeft  = 0.0f;
    outputRight = 0.0f;

    leftMotor.stop();
    rightMotor.stop();
  }
}