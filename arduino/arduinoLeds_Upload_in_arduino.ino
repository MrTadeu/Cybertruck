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
bool isIdle = true; // Flag to indicate idle state
uint8_t wavePosition = 0; // For the rainbow wave effect

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

    Serial.print("VARIABLE RECEIVED - ");
    Serial.print("vel_front ");
    Serial.print(vel_front);
    Serial.print(" pos_front ");
    Serial.print(pos_front);
    Serial.print(" vel_rear ");
    Serial.print(vel_rear);
    Serial.print(" pos_rear ");
    Serial.println(pos_rear);

    // Reset idle state if valid data is received
    isIdle = false;
  }
}

void loop() {
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    parseSerialData(msg);
  } else {
    isIdle = true; // No serial data received
  }

  if (isIdle || (vel_front == 0 && pos_front == 0)) {
    // Idle state: display rainbow wave on LED_PIN_1
    showRainbowWave();
  } else {
    // Update LED_PIN_1 based on vel_front
    updateStrip1();
  }

  // Update LED_PIN_2 for blinker logic
  updateStrip2();

  FastLED.show();
  delay(TRANSITION_DELAY);
}

// Display rainbow wave effect on LED_PIN_1
void showRainbowWave() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds1[i] = CHSV((wavePosition + i * 10) % 255, 255, 255); // Generate a rainbow wave
  }
  wavePosition++; // Increment the wave position for animation
}

// Update LED_PIN_1 based on vel_front with rainbow effect
void updateStrip1() {
  uint8_t hue = map(vel_front, -10, 10, 0, 255); // Map speed to hue (0 = green, 128 = yellow, 255 = red)
  CRGB color = CHSV(hue, 255, 255);              // Create a color from the hue
  fillStrip(leds1, color);
}

// Update LED_PIN_2 for blinker logic
void updateStrip2() {
  for (int i = 0; i < NUM_LEDS_2; i++) {
    leds2[i] = CRGB::White; // Set all LEDs to white by default
  }

  if (pos_front < 0) { // Left turn
    leds2[0] = CRGB(85, 255, 0); // Leftmost LED turns yellow
  } else if (pos_front > 0) { // Right turn
    leds2[3] = CRGB(85, 255, 0); // Rightmost LED turns yellow
  }
}

// Helper function to fill an LED strip with a single color
void fillStrip(CRGB* leds, CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
}
