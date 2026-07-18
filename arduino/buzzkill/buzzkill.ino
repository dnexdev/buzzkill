// buzzkill ESP32 firmware — receives ASCII commands over USB serial from QNX
// and drives the turret: pan/tilt servos, flywheel motors, pusher.
//
// Protocol (line-delimited, \n terminated):
//   A <pan> <tilt>   aim, signed degrees, ±90 (0 = center)
//   S <0|1>          spin flywheels off/on (leaves them running for fast follow-up shots)
//   F                fire one pusher pulse (assumes wheels are spun up)
//   D                disarm: center servos, stop wheels, retract pusher
//   H                heartbeat — resets the watchdog, no side effects
// Replies:
//   OK\n on success, ERR <reason>\n otherwise
//
// Safety: if no command is received for WATCHDOG_MS the firmware auto-disarms.
// Wire the flywheel MOSFET gate to FLYWHEEL_PIN. External servo/motor power required.

#include <ESP32Servo.h>

// ---- pin map — change to match your wiring ----
constexpr int PIN_PAN      = 13;
constexpr int PIN_TILT     = 12;
constexpr int PIN_PUSHER   = 14;   // pusher servo (or MOSFET gate for solenoid)
constexpr int PIN_FLYWHEEL = 27;   // MOSFET gate driving flywheel motors
constexpr int PIN_LED      = 2;    // onboard LED as status

// ---- servo geometry ----
constexpr int PAN_CENTER_US   = 1500;
constexpr int TILT_CENTER_US  = 1500;
constexpr int US_PER_DEG      = 10;   // ~1000-2000us over ±50°. Calibrate on-site.
constexpr int PAN_MIN_US      = 800;
constexpr int PAN_MAX_US      = 2200;
constexpr int TILT_MIN_US     = 900;
constexpr int TILT_MAX_US     = 2100;

constexpr int PUSHER_HOME_US  = 1000;
constexpr int PUSHER_PUSH_US  = 2000;
constexpr int PUSHER_PUSH_MS  = 120;   // how long to hold the push
constexpr int SHOT_COOLDOWN_MS = 250;  // min between pusher pulses

constexpr unsigned long WATCHDOG_MS = 500;

Servo pan_servo, tilt_servo, pusher_servo;

bool     flywheels_on   = false;
unsigned long last_cmd_ms = 0;
unsigned long last_shot_ms = 0;

void disarm() {
  pan_servo.writeMicroseconds(PAN_CENTER_US);
  tilt_servo.writeMicroseconds(TILT_CENTER_US);
  pusher_servo.writeMicroseconds(PUSHER_HOME_US);
  digitalWrite(PIN_FLYWHEEL, LOW);
  flywheels_on = false;
  digitalWrite(PIN_LED, LOW);
}

int clamp_us(int us, int lo, int hi) {
  return us < lo ? lo : (us > hi ? hi : us);
}

void do_aim(float pan_deg, float tilt_deg) {
  int p = PAN_CENTER_US  + (int)(pan_deg  * US_PER_DEG);
  int t = TILT_CENTER_US + (int)(tilt_deg * US_PER_DEG);
  pan_servo.writeMicroseconds(clamp_us(p, PAN_MIN_US, PAN_MAX_US));
  tilt_servo.writeMicroseconds(clamp_us(t, TILT_MIN_US, TILT_MAX_US));
}

void do_fire() {
  unsigned long now = millis();
  if (now - last_shot_ms < SHOT_COOLDOWN_MS) return;   // silently ignore rapid retriggers
  if (!flywheels_on) {
    digitalWrite(PIN_FLYWHEEL, HIGH);   // emergency spinup — but the shot will be weak
    flywheels_on = true;
    delay(200);                         // give wheels a moment (ideally the QNX side sent S1 earlier)
  }
  pusher_servo.writeMicroseconds(PUSHER_PUSH_US);
  delay(PUSHER_PUSH_MS);
  pusher_servo.writeMicroseconds(PUSHER_HOME_US);
  last_shot_ms = millis();
}

void handle_line(char* line) {
  last_cmd_ms = millis();
  digitalWrite(PIN_LED, HIGH);

  char cmd = line[0];
  char* rest = line + 1;
  while (*rest == ' ') ++rest;

  switch (cmd) {
    case 'A': {
      float pan = 0, tilt = 0;
      if (sscanf(rest, "%f %f", &pan, &tilt) != 2) { Serial.println("ERR aim"); return; }
      do_aim(pan, tilt);
      Serial.println("OK");
      break;
    }
    case 'S': {
      int on = 0;
      if (sscanf(rest, "%d", &on) != 1) { Serial.println("ERR spin"); return; }
      flywheels_on = (on != 0);
      digitalWrite(PIN_FLYWHEEL, flywheels_on ? HIGH : LOW);
      Serial.println("OK");
      break;
    }
    case 'F': do_fire();  Serial.println("OK"); break;
    case 'D': disarm();   Serial.println("OK"); break;
    case 'H': Serial.println("OK"); break;
    default:  Serial.println("ERR cmd");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_FLYWHEEL, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_FLYWHEEL, LOW);
  digitalWrite(PIN_LED, LOW);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  pan_servo.setPeriodHertz(50);
  tilt_servo.setPeriodHertz(50);
  pusher_servo.setPeriodHertz(50);
  pan_servo.attach(PIN_PAN,       PAN_MIN_US, PAN_MAX_US);
  tilt_servo.attach(PIN_TILT,     TILT_MIN_US, TILT_MAX_US);
  pusher_servo.attach(PIN_PUSHER, PUSHER_HOME_US, PUSHER_PUSH_US);

  disarm();
  Serial.println("READY");
  last_cmd_ms = millis();
}

char buf[64];
size_t buf_len = 0;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf_len == 0) continue;
      buf[buf_len] = '\0';
      handle_line(buf);
      buf_len = 0;
    } else if (buf_len < sizeof(buf) - 1) {
      buf[buf_len++] = c;
    } else {
      buf_len = 0;   // overflow, drop line
      Serial.println("ERR ovf");
    }
  }
  if (millis() - last_cmd_ms > WATCHDOG_MS) {
    disarm();
  }
}
