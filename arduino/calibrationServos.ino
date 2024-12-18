//  ------> http://www.adafruit.com/products/815
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// 270º porem só vamos usar 35 para um lado + 25 para outro
// MEIO 277 !!!!!!
#define SERVOMIN  70//253 // 70 -se fosse 270º 
#define SERVOMAX  500//332 // 500 -se fosse 270º 
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// 280 meio

uint8_t leftServo = 0;
uint8_t rightServo = 1;
int data = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


  askServoPWM();

  delay(10);
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void askServoPWM() {
  Serial.println("PWM (volor de 0 a 270): ");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Lê a linha inteira até encontrar uma quebra de linha
    data = input.toInt();

    // 330 277 240

    // int pulso = map(data, 0, 270, SERVOMIN, SERVOMAX); // 135
    int pulso = data; //map(data, -20, 30, SERVOMIN, SERVOMAX); /* -25, 35 */ // 165, 115

    pwm.setPWM(leftServo, 0, pulso);
    pwm.setPWM(rightServo, 0, pulso + 4);
    Serial.print("pwm "); Serial.println(pulso);
  }
  /* int pulso = map(30, -20, 30, SERVOMIN, SERVOMAX);
  pwm.setPWM(leftServo, 0, pulso);
  pwm.setPWM(rightServo, 0, pulso);

  delay(250);
  pulso = map(-20, -20, 30, SERVOMIN, SERVOMAX);
  pwm.setPWM(leftServo, 0, pulso);
  pwm.setPWM(rightServo, 0, pulso); */
  /* // pwm.setPWM(leftServo, 0, 600);
  for (uint16_t pulselen = 0; pulselen < 500; pulselen=pulselen+20) {
    Serial.print("pwmlen "); Serial.println(pulselen);
    pwm.setPWM(leftServo, 0, pulselen);
    delay(1000);
  } 
  Serial.println("-----------------------------------------------------");

  for (uint16_t pulselen = SERVOMAX; pulselen >= 70; pulselen=pulselen-10) {
    Serial.print("pwmlen "); Serial.println(pulselen);
    pwm.setPWM(leftServo, 0, pulselen);
    delay(1000);
  } */

  delay(500);
}