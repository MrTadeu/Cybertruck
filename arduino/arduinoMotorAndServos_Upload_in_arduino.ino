#include <Servo.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


//servo motor
#define SERVOMID 277
#define SERVOMIN 250
#define SERVOMAX 310

#define SERVO_FREQ 50 

#define LEFT_SERVO 0 // FRONT 
#define RIGHT_SERVO 1

int pulsoFront = SERVOMID;
int pulsoRear = SERVOMID;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


//brushless motor
#define STOP_PULSE_LENGTH 1000 // Neutral/Stop pulse length in µs
#define MIN_PULSE_LENGTH 1150 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 1350 //DEFAULT - 2000 / SAFE - 1350 // Maximum pulse in µs

#define PULSE_CLOCKWISE 1000 // 0-50% = 1000 - 1500 FRONT - tipo uma flag booleana
#define PULSE_REVERSE_CLOCKWISE 2000 // 50-100% = 1500 -2000 REVERSE - tipo uma flag booleana

Servo motA, motB, motAR, motBR;

// Definição das variáveis para armazenar os valores
float vel_front = 0;
float pos_front = 0;
float vel_rear = 0;
float pos_rear = 0;

bool isValidMessage(const String &msg)
{
    return msg.indexOf("vel_front") != -1 &&
           msg.indexOf("pos_front") != -1 &&
           msg.indexOf("vel_rear") != -1 &&
           msg.indexOf("pos_rear") != -1;
}

void parseSerialData(String input)
{
    // Verifica se o input contém os parâmetros esperados
    if (isValidMessage(input))
    {
        int velFrontStart = input.indexOf("vel_front") + 10;
        int posFrontStart = input.indexOf("pos_front") + 10;
        int velRearStart = input.indexOf("vel_rear") + 9;
        int posRearStart = input.indexOf("pos_rear") + 9;

        vel_front = input.substring(velFrontStart, input.indexOf(" ", velFrontStart)).toFloat();
        pos_front = input.substring(posFrontStart, input.indexOf(" ", posFrontStart)).toFloat();
        vel_rear = input.substring(velRearStart, input.indexOf(" ", velRearStart)).toFloat();
        pos_rear = input.substring(posRearStart).toFloat();

        // Envia feedback para o ROS2
        Serial.print("VARIABLE RECIVED- ");
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

void setThrottle(int pulseLength, int clock_wise) {
    motA.writeMicroseconds(pulseLength);
    motB.writeMicroseconds(pulseLength);
    if(clock_wise == 1){
      motAR.writeMicroseconds(PULSE_CLOCKWISE);
      motBR.writeMicroseconds(PULSE_CLOCKWISE);
    }
    else{
      motAR.writeMicroseconds(PULSE_REVERSE_CLOCKWISE);
      motBR.writeMicroseconds(PULSE_REVERSE_CLOCKWISE);
    }
}

void setup()
{
    // Serial ROS2
    Serial.begin(115200); // 115200 baud
    while (!Serial);

    // servo motor
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  

    // brushless motor
    motA.attach(2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motAR.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motBR.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

    setThrottle(STOP_PULSE_LENGTH,0);

    Serial.println("Arduino pronto para receber comandos via ROS2!");
}

void loop()
{
    // Serial.println("vel_front 100 pos_front 0 vel_rear 50 pos_rear 0");

    if (Serial.available() > 0) {
      String msg = Serial.readStringUntil('\n'); // Ler até "\r" (final da mensagem)
      msg.trim();
      Serial.println(msg);
      parseSerialData(msg);

      if(vel_front == 0){
          setThrottle(STOP_PULSE_LENGTH, 0);
      }
      else if(vel_front > 0){
          setThrottle(map(vel_front,0.01,10,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH), 0);
      }
      else if(vel_front < 0){
          setThrottle(map(vel_front,-0.01, -10,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH), 1);
      }
      
      if (pos_front == 0 ) {
        pulsoFront = SERVOMID;
        pulsoRear = SERVOMID;
      } 
      else if (pos_front > 0) {
        pulsoFront = map(pos_front,0.01,2.00, SERVOMID, SERVOMIN);
        pulsoRear = map(pos_front,0.01,2.00, SERVOMID, SERVOMAX);
      }
      else if (pos_front < 0) {
        pulsoFront = map(pos_front,-0.01,-2.00, SERVOMID, SERVOMAX);
        pulsoRear = map(pos_front,-0.01,-2.00, SERVOMID, SERVOMIN);
      }
      pwm.setPWM(LEFT_SERVO, 0, pulsoFront);
      pwm.setPWM(RIGHT_SERVO, 0, pulsoRear + 4);
    }
}
