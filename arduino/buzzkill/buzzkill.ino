// Buzzkill turret firmware for an ESP32 DevKit.
//
// The command interface is intentionally line-oriented so it works from both
// the Arduino Serial Monitor and a Raspberry Pi/QNX serial process.

// IMPORTANT: Power the servos from a regulated external 5 V supply. Connect
// the servo supply ground to ESP32 ground. Drive the flywheel through a proper
// logic-level N-channel MOSFET and flyback protection suitable for the motor.

#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// Hardware and tuning configuration
// ---------------------------------------------------------------------------

constexpr int PAN_PIN = 13;
constexpr int TILT_PIN = 12;
constexpr int PUSHER_PIN = 14;
constexpr int FLYWHEEL_PIN = 27;

constexpr int CENTER_PAN = 90;
constexpr int CENTER_TILT = 90;

constexpr int PAN_MIN = 30;
constexpr int PAN_MAX = 150;
constexpr int TILT_MIN = 45;
constexpr int TILT_MAX = 135;

constexpr int PUSHER_FORWARD = 150;
constexpr int PUSHER_BACK = 30;

constexpr unsigned long FLYWHEEL_SPINUP_MS = 800;
constexpr unsigned long PUSHER_FORWARD_MS = 180;
constexpr unsigned long PUSHER_SETTLE_MS = 180;

// Delay in milliseconds between one-degree servo steps. Lower is faster.
constexpr unsigned long SERVO_SPEED = 12;

constexpr size_t SERIAL_BUFFER_SIZE = 64;

Servo panServo;
Servo tiltServo;
Servo pusherServo;

int currentPan = CENTER_PAN;
int currentTilt = CENTER_TILT;
int targetPan = CENTER_PAN;
int targetTilt = CENTER_TILT;

bool armed = false;
bool flywheelOn = false;
unsigned long lastServoStepMs = 0;

enum class FireState { IDLE, SPINUP, PUSHING, RETRACTING };
FireState fireState = FireState::IDLE;
unsigned long fireStateStartedMs = 0;
bool restoreFlywheelOff = false;

char serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialLength = 0;
bool discardingSerialLine = false;

void handleSerial();
void movePanTilt(int panOffset, int tiltOffset);
void updatePanTilt();
void setFlywheel(bool enabled);
void fireOnce();
void updateFiring();
void cancelFiring();
void arm();
void disarm();
void emergencyStop();
void printHelp();
void printAimStatus();

void setup() {
  Serial.begin(115200);

  pinMode(FLYWHEEL_PIN, OUTPUT);
  digitalWrite(FLYWHEEL_PIN, LOW);

  // Allocate separate ESP32 PWM timers for predictable servo output.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  pusherServo.setPeriodHertz(50);

  panServo.attach(PAN_PIN, 500, 2500);
  tiltServo.attach(TILT_PIN, 500, 2500);
  pusherServo.attach(PUSHER_PIN, 500, 2500);

  panServo.write(CENTER_PAN);
  tiltServo.write(CENTER_TILT);
  pusherServo.write(PUSHER_BACK);
  setFlywheel(false);

  Serial.println();
  Serial.println("Buzzkill turret ready (DISARMED)");
  printHelp();
}

void loop() {
  handleSerial();
  updatePanTilt();
  updateFiring();
}

// Collect one newline-terminated command without blocking the control loop.
void handleSerial() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (discardingSerialLine) {
      if (incoming == '\r' || incoming == '\n') {
        discardingSerialLine = false;
      }
      continue;
    }

    if (incoming == '\r' || incoming == '\n') {
      if (serialLength == 0) {
        continue;
      }

      serialBuffer[serialLength] = '\0';
      serialLength = 0;

      char command = '\0';
      int panOffset = 0;
      int tiltOffset = 0;
      char extra = '\0';

      // Aim is the only command that accepts arguments. The extra conversion
      // rejects trailing junk such as "A 10 20 xyz".
      if (serialBuffer[0] == 'A') {
        if (sscanf(serialBuffer, " %c %d %d %c", &command, &panOffset,
                   &tiltOffset, &extra) != 3) {
          Serial.println("ERR: expected A <panOffset> <tiltOffset>");
          continue;
        }
        movePanTilt(panOffset, tiltOffset);
        continue;
      }

      // All remaining commands must be exactly one non-whitespace character.
      if (sscanf(serialBuffer, " %c %c", &command, &extra) != 1) {
        Serial.println("ERR: malformed command (H for help)");
        continue;
      }

      switch (command) {
        case 'F':
          if (!armed) {
            Serial.println("IGNORED: system is disarmed");
          } else {
            setFlywheel(true);
            // An explicit ON command becomes the state preserved after a shot.
            restoreFlywheelOff = false;
          }
          break;
        case 'f':
          if (fireState != FireState::IDLE) {
            cancelFiring();
            Serial.println("Firing cycle: cancelled");
          }
          setFlywheel(false);
          break;
        case 'P':
          fireOnce();
          break;
        case 'S':
          emergencyStop();
          break;
        case 'D':
          disarm();
          break;
        case 'E':
          arm();
          break;
        case 'H':
          printHelp();
          break;
        default:
          Serial.println("ERR: unknown command (H for help)");
          break;
      }
    } else if (serialLength < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[serialLength++] = incoming;
    } else {
      // Drop an overlong line, including bytes that arrive in a later loop.
      serialLength = 0;
      discardingSerialLine = true;
      Serial.println("ERR: command too long");
    }
  }
}

// Set safe targets. updatePanTilt() performs the actual gradual movement.
void movePanTilt(int panOffset, int tiltOffset) {
  targetPan = constrain(CENTER_PAN + panOffset, PAN_MIN, PAN_MAX);
  targetTilt = constrain(CENTER_TILT + tiltOffset, TILT_MIN, TILT_MAX);

  Serial.print("Moving to Pan: ");
  Serial.print(targetPan);
  Serial.print(" deg, Tilt: ");
  Serial.print(targetTilt);
  Serial.println(" deg");

  if (currentPan == targetPan && currentTilt == targetTilt) {
    printAimStatus();
  }
}

void updatePanTilt() {
  const unsigned long now = millis();
  if (now - lastServoStepMs < SERVO_SPEED) {
    return;
  }
  lastServoStepMs = now;

  bool moved = false;
  if (currentPan != targetPan) {
    currentPan += (targetPan > currentPan) ? 1 : -1;
    currentPan = constrain(currentPan, PAN_MIN, PAN_MAX);
    panServo.write(currentPan);
    moved = true;
  }
  if (currentTilt != targetTilt) {
    currentTilt += (targetTilt > currentTilt) ? 1 : -1;
    currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);
    tiltServo.write(currentTilt);
    moved = true;
  }

  static bool wasMoving = false;
  if (!moved && wasMoving) {
    printAimStatus();
  }
  wasMoving = moved;
}

void setFlywheel(bool enabled) {
  flywheelOn = enabled;
  digitalWrite(FLYWHEEL_PIN, enabled ? HIGH : LOW);
  Serial.println(enabled ? "Flywheel: ON" : "Flywheel: OFF");
}

void fireOnce() {
  if (!armed) {
    Serial.println("IGNORED: system is disarmed");
    return;
  }
  if (fireState != FireState::IDLE) {
    Serial.println("IGNORED: firing cycle already active");
    return;
  }

  restoreFlywheelOff = !flywheelOn;
  fireStateStartedMs = millis();
  if (restoreFlywheelOff) {
    setFlywheel(true);
    Serial.println("Firing: spinning up");
    fireState = FireState::SPINUP;
  } else {
    Serial.println("Firing: push");
    pusherServo.write(PUSHER_FORWARD);
    fireState = FireState::PUSHING;
  }
}

void updateFiring() {
  if (fireState == FireState::IDLE) {
    return;
  }

  const unsigned long now = millis();
  const unsigned long elapsed = now - fireStateStartedMs;

  if (fireState == FireState::SPINUP && elapsed >= FLYWHEEL_SPINUP_MS) {
    Serial.println("Firing: push");
    pusherServo.write(PUSHER_FORWARD);
    fireState = FireState::PUSHING;
    fireStateStartedMs = now;
  } else if (fireState == FireState::PUSHING && elapsed >= PUSHER_FORWARD_MS) {
    pusherServo.write(PUSHER_BACK);
    fireState = FireState::RETRACTING;
    fireStateStartedMs = now;
  } else if (fireState == FireState::RETRACTING && elapsed >= PUSHER_SETTLE_MS) {
    if (restoreFlywheelOff) {
      setFlywheel(false);
    }
    fireState = FireState::IDLE;
    Serial.println("Firing cycle: complete");
  }
}

void cancelFiring() {
  pusherServo.write(PUSHER_BACK);
  fireState = FireState::IDLE;
  restoreFlywheelOff = false;
}

void arm() {
  armed = true;
  Serial.println("System: ARMED");
}

void disarm() {
  armed = false;
  cancelFiring();
  setFlywheel(false);
  Serial.println("System: DISARMED");
}

void emergencyStop() {
  armed = false;
  targetPan = CENTER_PAN;
  targetTilt = CENTER_TILT;
  cancelFiring();
  setFlywheel(false);
  Serial.println("EMERGENCY STOP: disarmed, pusher retracted, centering");
}

void printAimStatus() {
  Serial.print("Pan: ");
  Serial.print(currentPan);
  Serial.println(" deg");
  Serial.print("Tilt: ");
  Serial.print(currentTilt);
  Serial.println(" deg");
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  A <panOffset> <tiltOffset>  Smooth aim relative to center");
  Serial.println("  F                           Flywheel ON");
  Serial.println("  f                           Flywheel OFF");
  Serial.println("  P                           Fire once (armed only)");
  Serial.println("  S                           Emergency stop and center");
  Serial.println("  D                           Disarm");
  Serial.println("  E                           Arm");
  Serial.println("  H                           Show this help");
}
