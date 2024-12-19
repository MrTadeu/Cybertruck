#include <FastLED.h>

// Configuration
#define LED_PIN_1 2         // Pin connected to the first WS2812B strip (D2)
#define LED_PIN_2 3         // Pin connected to the second WS2812B strip (D3)
#define NUM_LEDS 30         // Number of LEDs in each strip
#define BRIGHTNESS 255      // Brightness of LEDs (0-255)
#define TRANSITION_DELAY 10 // Delay in milliseconds between updates

CRGB leds1[NUM_LEDS]; // Define the LED array for the first strip
CRGB leds2[NUM_LEDS]; // Define the LED array for the second strip

void setup() {
  // Initialize the LED strips
  FastLED.addLeds<WS2812B, LED_PIN_1, GRB>(leds1, NUM_LEDS);
  FastLED.addLeds<WS2812B, LED_PIN_2, GRB>(leds2, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
  // Colors
  CRGB green = CRGB::Green;
  CRGB red = CRGB::Red;

  // Transition from green to red
  for (float blendFactor = 0.0; blendFactor <= 1.0; blendFactor += 0.01) {
    CRGB blendedColor = blendColors(green, red, blendFactor);
    fillStrip(leds1, blendedColor); // Update the first strip
    fillStrip(leds2, blendedColor); // Update the second strip
    delay(TRANSITION_DELAY);
  }

  // Transition from red to green
  for (float blendFactor = 1.0; blendFactor >= 0.0; blendFactor -= 0.01) {
    CRGB blendedColor = blendColors(green, red, blendFactor);
    fillStrip(leds1, blendedColor); // Update the first strip
    fillStrip(leds2, blendedColor); // Update the second strip
    delay(TRANSITION_DELAY);
  }
}

// Helper function to blend two colors
CRGB blendColors(CRGB color1, CRGB color2, float blendFactor) {
  return color1.lerp8(color2, blendFactor * 255); // lerp8 interpolates between two colors
}

// Helper function to set all LEDs in a strip to the same color
void fillStrip(CRGB* leds, CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}
