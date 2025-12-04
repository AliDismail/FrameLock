#include <IRremote.h>
#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// ---- LED ring config ----
#define LED_PIN    6
#define LED_COUNT  24

// ---- Servo config ----
#define SERVO_PIN      9
#define SERVO_DOWN_POS 90
#define SERVO_UP_POS   0

// ---- IR config ----
const int RECV_PIN = 11;

BH1750 lightMeter;
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo armServo;

// Threshold
const float DARK_THRESHOLD_LUX = 150.0;

// Track state
bool lightIsOn = false;

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);

  Wire.begin();
  if (lightMeter.begin()) {
    Serial.println("BH1750 started OK");
  } else {
    Serial.println("BH1750 error – check wiring");
  }

  ring.begin();
  ring.setBrightness(100);
  ring.show();

  armServo.attach(SERVO_PIN);
  armServo.write(SERVO_DOWN_POS);
}

void loop() {
  // --- IR remote check ---
  if (IrReceiver.decode()) {
    handleIRCommand(IrReceiver.decodedIRData.command);
    IrReceiver.resume(); // Ready for next code
  }

  // --- Brightness control (always active) ---
  handleBrightnessControl();

  delay(200); // small delay to avoid spamming
}

// ---------------- Helper functions ----------------
void handleIRCommand(uint8_t command) {
  switch (command) {
    case 22:  Serial.println("KEY_1"); break;
    case 25:  Serial.println("KEY_2"); break;
    case 13:  Serial.println("KEY_3"); break;
    case 12:  Serial.println("KEY_4"); break;
    case 24:  Serial.println("KEY_5"); break;
    case 94:  Serial.println("KEY_6"); break;
    case 8:   Serial.println("KEY_7"); break;
    case 28:  Serial.println("KEY_8"); break;
    case 90:  Serial.println("KEY_9"); break;
    case 82:  Serial.println("KEY_0"); break;
    default:  Serial.print("Unknown command: ");
              Serial.println(command);
              break;
  }
}

// Brightness control logic
void handleBrightnessControl() {
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");

  if (lux < DARK_THRESHOLD_LUX && !lightIsOn) {
    turnRingOnWhite();
    raiseLight();
    lightIsOn = true;
  } else if (lux >= DARK_THRESHOLD_LUX && lightIsOn) {
    turnRingOff();
    lowerLight();
    lightIsOn = false;
  }
}

// LED helpers
void turnRingOnWhite() {
  for (int i = 0; i < LED_COUNT; i++) {
    ring.setPixelColor(i, ring.Color(255, 255, 255));
  }
  ring.show();
}

void turnRingOff() {
  ring.clear();
  ring.show();
}

// Servo helpers
void raiseLight() {
  armServo.write(SERVO_UP_POS);
  delay(500);
}

void lowerLight() {
  armServo.write(SERVO_DOWN_POS);
  delay(500);
}
