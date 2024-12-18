int vel_front = 0;
int pos_front = 0;
int vel_rear = 0;
int pos_rear = 0;

int decay_vel = 5;
int decay_pos = 5;

void setup()
{
    // Inicializa a Serial para comunicação com ROS2
    Serial.begin(115200); // Configura a Serial para 115200 baud
    while (!Serial)
        ; // Aguarda inicialização completa da Serial

    Serial.println("Arduino pronto para receber comandos via ROS2!");
}

void loop()
{
    // Verifica se há dados disponíveis na Serial
    // Serial.println("vel_front 100 pos_front 0 vel_rear 50 pos_rear 0");

    if (Serial.available() > 0)
    {
        String msg = Serial.readStringUntil('\r'); // Ler até "\r" (final da mensagem)
        Serial.println("Received: " + msg);
        // processInput(msg);                         // Processa a mensagem recebida
        // vel_front - decay_vel == 0 ? 0 : vel_front - decay_vel;
        // pos_front - decay_pos == 0 ? 0 : pos_front - decay_pos;
        // vel_rear - decay_vel == 0 ? 0 : vel_rear - decay_vel;
        // pos_rear - decay_pos == 0 ? 0 : pos_rear - decay_pos;
    }
    delay(500);
}

// Função para processar o comando recebido
void processInput(String input)
{
    // Variáveis temporárias para armazenar índices
    int vel_front_pos = input.indexOf("vel_front");
    int pos_front_pos = input.indexOf("pos_front");
    int vel_rear_pos = input.indexOf("vel_rear");
    int pos_rear_pos = input.indexOf("pos_rear");

    // Extrai e converte os valores
    if (vel_front_pos != -1)
    {
        vel_front = input.substring(vel_front_pos + 9, input.indexOf(' ', vel_front_pos + 9)).toInt();
    }
    if (pos_front_pos != -1)
    {
        pos_front = input.substring(pos_front_pos + 9, input.indexOf(' ', pos_front_pos + 9)).toInt();
    }
    if (vel_rear_pos != -1)
    {
        vel_rear = input.substring(vel_rear_pos + 8, input.indexOf(' ', vel_rear_pos + 8)).toInt();
    }
    if (pos_rear_pos != -1)
    {
        pos_rear = input.substring(pos_rear_pos + 8, input.indexOf(' ', pos_rear_pos + 8)).toInt();
    }

    // Envia feedback para o ROS2
    Serial.print("vel_front ");
    Serial.print(vel_front);
    Serial.print(" pos_front ");
    Serial.print(pos_front);
    Serial.print(" vel_rear ");
    Serial.print(vel_rear);
    Serial.print(" pos_rear ");
    Serial.println(pos_rear);
}
