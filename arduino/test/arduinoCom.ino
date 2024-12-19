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
void setup()
{
    // Serial ROS2
    Serial.begin(115200); // 115200 baud
    while (!Serial);

    Serial.println("Arduino pronto para receber comandos via ROS2!");
}

void loop()
{
    // Serial.println("vel_front 100 pos_front 0 vel_rear 50 pos_rear 0");

    if (Serial.available() > 0)
    {
        String msg = Serial.readStringUntil('\n'); // Ler até "\r" (final da mensagem)
        msg.trim();
        Serial.println(msg);
        parseSerialData(msg);
        // vel_front - decay_vel == 0 ? 0 : vel_front - decay_vel;
        // pos_front - decay_pos == 0 ? 0 : pos_front - decay_pos;
        // vel_rear - decay_vel == 0 ? 0 : vel_rear - decay_vel;
        // pos_rear - decay_pos == 0 ? 0 : pos_rear - decay_pos;
    }
    delay(500);
}
