#include <FastLED.h>

// Configuration
#define LED_PIN_1 2         // Pin connected to the first WS2812B strip (D2)
#define LED_PIN_2 3         // Pin connected to the second WS2812B strip (D3)
#define NUM_LEDS 30         // Number of LEDs in the first strip
#define NUM_LEDS_2 4        // Number of LEDs in the second strip
#define BRIGHTNESS 255      // Brightness of LEDs (0-255)
#define TRANSITION_DELAY 10 // Delay in milliseconds between updates

CRGB leds1[NUM_LEDS];       // Define the LED array for the first strip
CRGB leds2[NUM_LEDS_2];     // Define the LED array for the second strip

// Variables
float vel_front = 0;
float pos_front = 0;
float vel_rear = 0;
float pos_rear = 0;

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2812B, LED_PIN_1, GRB>(leds1, NUM_LEDS);
  FastLED.addLeds<WS2812B, LED_PIN_2, GRB>(leds2, NUM_LEDS_2);
  FastLED.setBrightness(BRIGHTNESS);

  while (!Serial);
}

bool isValidMessage(const String &msg) {
  return msg.indexOf("vel_front") != -1 &&
         msg.indexOf("pos_front") != -1 &&
         msg.indexOf("vel_rear") != -1 &&
         msg.indexOf("pos_rear") != -1;
}

void parseSerialData(String input) {
  if (isValidMessage(input)) {
    int velFrontStart = input.indexOf("vel_front") + 10;
    int posFrontStart = input.indexOf("pos_front") + 10;
    int velRearStart = input.indexOf("vel_rear") + 9;
    int posRearStart = input.indexOf("pos_rear") + 9;

    vel_front = input.substring(velFrontStart, input.indexOf(" ", velFrontStart)).toFloat();
    pos_front = input.substring(posFrontStart, input.indexOf(" ", posFrontStart)).toFloat();
    vel_rear = input.substring(velRearStart, input.indexOf(" ", velRearStart)).toFloat();
    pos_rear = input.substring(posRearStart).toFloat();

    Serial.print("VARIABLE RECIEVED- ");
    Serial.print("vel_front ");
    Serial.print(vel_front);
    Serial.print(" pos_front ");
    Serial.print(pos_front);
    Serial.print(" vel_rear ");
    Serial.print(vel_rear);
    Serial.print(" pos_rear ");
    Serial.println(pos_rear);
  }
}

void loop() {
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    parseSerialData(msg);
  }

  // Update LED_PIN_1 based on vel_front
  updateStrip1();

  // Update LED_PIN_2 for turn signals
  updateStrip2();

  FastLED.show();
  delay(TRANSITION_DELAY);
}

// Update LED_PIN_1 based on vel_front
void updateStrip1() {
  CRGB color;
  if (vel_front > 0) {
    color = CRGB::Green.lerp8(CRGB::DarkGreen, map(vel_front, 0, 10, 0, 255));
  } else if (vel_front < 0) {
    color = CRGB::Red.lerp8(CRGB::DarkRed, map(abs(vel_front), 0, 10, 0, 255));
  } else {
    color = CRGB::Black; // Off when stopped
  }
  fillStrip(leds1, color);
}

// Update LED_PIN_2 for turn signals
void updateStrip2() {
  static bool blinkState = false;
  blinkState = !blinkState; // Toggle blink state

  leds2[1] = CRGB::White; // Center LEDs always white
  leds2[2] = CRGB::White;

  if (pos_front < 0) { // Turning left
    leds2[0] = blinkState ? CRGB::Yellow : CRGB::Black;
    leds2[3] = CRGB::Black;
  } else if (pos_front > 0) { // Turning right
    leds2[0] = CRGB::Black;
    leds2[3] = blinkState ? CRGB::Yellow : CRGB::Black;
  } else { // No turn
    leds2[0] = CRGB::Black;
    leds2[3] = CRGB::Black;
  }
}

// Helper function to fill an LED strip with a single color
void fillStrip(CRGB* leds, CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
}
