#include <DHT.h>
#include <PID_v1.h>

#define DHT_PIN 2      // Pino de dados do sensor DHT22
#define DHT_TYPE DHT22 // Tipo de sensor DHT

DHT dht(DHT_PIN, DHT_TYPE);

// Configuração do PID
double setpoint = 60.0; // Umidade desejada em percentagem
double input, output;
double Kp = 5; // Ganho proporcional
double Ki = 0.1; // Ganho integral
double Kd = 1; // Ganho derivativo

double elapsedTime, previousTime;
double error, lastError;
double cumError, rateError;

// Definindo os limites de saída do PID
double outMin = 0;
double outMax = 100;

// Pinagem do dispositivo de controle de umidade (um relé, por exemplo)
// Substitua pelo pino que controla o dispositivo em seu hardware específico
const int CONTROL_PIN = 13;

void setup() {
  Serial.begin(9600);
  dht.begin();

  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, LOW);
}

void loop() {
  double currentHumidity = dht.readHumidity();

  // Chama a função do PID
  PID_Controller(currentHumidity);

  // Exibe informações no Serial Monitor
  Serial.print("Umidade Atual: ");
  Serial.print(currentHumidity);
  Serial.print(" % | Saída do PID: ");
  Serial.println(output);

  delay(1000); // Aguarda um segundo antes de realizar a próxima leitura
}

void PID_Controller(double currentHumidity) {
  // Calcula o erro
  error = setpoint - currentHumidity;

  // Calcula o tempo decorrido
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  // Calcula as componentes PID
  double P = Kp * error;
  cumError += error * elapsedTime;
  double I = Ki * cumError;
  rateError = (error - lastError) / elapsedTime;
  double D = Kd * rateError;

  // Calcula a saída PID
  output = P + I + D;

  // Limita a saída do PID dentro dos limites especificados
  output = constrain(output, outMin, outMax);

  // Aplica a saída do PID para controlar algum dispositivo (por exemplo, um umidificador)
  // Neste exemplo, apenas exibimos a saída no Serial Monitor
  // Você deve adaptar esta parte para controlar seu sistema específico
  // (por exemplo, controle de um relé para ligar/desligar um umidificador)
  digitalWrite(CONTROL_PIN, output > 50); // Liga o dispositivo se a saída for maior que 50

  // Atualiza variáveis para a próxima iteração
  lastError = error;
  previousTime = currentTime;
}
