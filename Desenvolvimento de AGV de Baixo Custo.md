//*
 * * NOTA TÉCNICA:
 * O ESP32 opera com lógica de 3.3V. 
 * O pino ECHO do HC-SR04 (5V) requer divisor de tensão para o GPIO do ESP32.
 *//

#include <Arduino.h>

// --- MAPEAMENTO DE HARDWARE (ESP32 DevKit V1) ---

    // Ponte H L298N - Motor A (Esquerdo)
  const int IN1 = 27;  // GPIO 27
  const int IN2 = 26;  // GPIO 26
  const int ENA = 14;  // GPIO 14 (PWM)

// Ponte H L298N - Motor B (Direito)
  const int IN3 = 25;  // GPIO 25
  const int IN4 = 33;  // GPIO 33
  const int ENB = 32;  // GPIO 32 (PWM)

// Sensores
  const int TRIG_PIN = 5;   // GPIO 5 (Saída Trigger)
  const int ECHO_PIN = 18;  // GPIO 18 (Entrada Echo - CUIDADO: Usar divisor de tensão)
  const int IR_SENSOR = 19; // GPIO 19 (Entrada IR)

// Comunicação Bluetooth (Serial 2 no ESP32)
// RX2 padrão é GPIO 16 | TX2 padrão é GPIO 17
#define RXD2 16
#define TXD2 17

// --- VARIÁVEIS DE CONTROLE ---
// PWM no ESP32 é de 0 a 255 (8 bits por padrão no Arduino Core)
  const int VELOCIDADE_PADRAO = 180; 
  const int VELOCIDADE_CURVA = 150;
  const int DISTANCIA_STOP = 20; // cm

long duration;
int distance;
char comando = 'S'; // Estado inicial: Parado

void setup() 
{
  // Inicialização Serial (Monitor Serial do PC)
  Serial.begin(115200);
  
  // Inicialização Bluetooth (HC-05)
  // Serial2.begin(BaudRate, Protocolo, RX_Pin, TX_Pin);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Configuração dos Pinos
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_SENSOR, INPUT);

  Serial.println(">>> SISTEMA ESP32 AGV INICIADO <<<");
  Serial.println("Aguardando conexão Bluetooth...");
}

void loop() 
{
  // --- 1. LEITURA DE SENSORES ---
  
  // HC-SR04: Gera pulso de 10us
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Leitura do pulso de retorno
  duration = pulseIn(ECHO_PIN, HIGH);
  // Cálculo da distância (cm)
  distance = duration * 0.034 / 2;

  // Leitura do Sensor IR (Lógica Invertida comum: LOW = Obstáculo detectado)
  bool obstaculoIR = !digitalRead(IR_SENSOR); 

  // --- 2. LÓGICA DE SEGURANÇA E CONTROLE ---
  
  // Verifica se recebeu comando via Bluetooth
    if (Serial2.available()) 
      {
        comando = Serial2.read();
        Serial.print("Comando: ");
        Serial.println(comando);
      }

  // INTERTRAVAMENTO DE SEGURANÇA
  // Se detectar obstáculo, força parada (exceto se o comando for Ré 'B')
    if ((distance > 0 && distance < DISTANCIA_STOP) || obstaculoIR)  
      {
        if (comando == 'F') 
        { 
        // Se tentou ir para frente com obstáculo
          parar();
          Serial.println("[ALERTA] Obstáculo detectado! Parada forçada.");
        } 
        else 
        {
        // Permite manobras (Ré, Giros) para sair do obstáculo
          executarMovimento(comando); 
        }
      } 
    else 
      {
      // Caminho livre, executa comando normal
        executarMovimento(comando);
      }
  
  delay(50);   // Estabilidade do loop
}

  // --- FUNÇÃO DE ATUAÇÃO DOS MOTORES ---
    void executarMovimento(char cmd) 
  {
    switch (cmd) 
      {
        case 'F':   // Frente
        controlarMotor(VELOCIDADE_PADRAO, VELOCIDADE_PADRAO, HIGH, LOW, HIGH, LOW);
        break;
        case 'B': // Ré (Back)
        controlarMotor(VELOCIDADE_PADRAO, VELOCIDADE_PADRAO, LOW, HIGH, LOW, HIGH);
        break;
        case 'L': // Esquerda (Giro no eixo)
        controlarMotor(VELOCIDADE_CURVA, VELOCIDADE_CURVA, LOW, HIGH, HIGH, LOW);
        break;
        case 'R': // Direita (Giro no eixo)
        controlarMotor(VELOCIDADE_CURVA, VELOCIDADE_CURVA, HIGH, LOW, LOW, HIGH);
        break;
        case 'S': // Stop
        default:
        parar();
        break;
      }
  }

  // Função auxiliar para reduzir código repetitivo
void controlarMotor(int velA, int velB, int in1, int in2, int in3, int in4) 
  {
  // Motor A (Esquerdo)
    digitalWrite(IN1, in1);
    digitalWrite(IN2, in2);
    analogWrite(ENA, velA); // ESP32 converte isso para PWM automaticamente

  // Motor B (Direito)
    digitalWrite(IN3, in3);
    digitalWrite(IN4, in4);
    analogWrite(ENB, velB);
  }

void parar() 
  {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
  }
