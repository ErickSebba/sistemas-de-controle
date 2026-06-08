#include <Encoder.h>

//POSIÇÕES E VELOCIDADES

// Pinos do Encoder do Motor (Braço)
const int motorEncoderPinA = 18;   // Fio Amarelo
const int motorEncoderPinB = 19;   // Fio Verde

// Pinos do Encoder do Pêndulo
const int pendulumEncoderPinA = 20; // Fio Branco
const int pendulumEncoderPinB = 21; // Fio Verde

// Cria os objetos encoder para o motor e o pêndulo
Encoder motorEncoder(motorEncoderPinA, motorEncoderPinB);
Encoder pendulumEncoder(pendulumEncoderPinA, pendulumEncoderPinB);

// Constantes de conversão para posições em graus e radianos
const float pulsesPerDegreePendulum = 6.6667;
const float pulsesPerDegreeMotor = 1.1667;
const float pulsesPerRadianPendulum = pulsesPerDegreePendulum * 57.2958;
const float pulsesPerRadianMotor = pulsesPerDegreeMotor * 57.2958;

// Variáveis para armazenar posições e velocidades
float motorPositionDegrees = 0, motorPositionRadians = 0;
float pendulumPositionDegrees = 0, pendulumPositionRadians = 0;
float motorVelocityDegrees = 0, motorVelocityRadians = 0;
float pendulumVelocityDegrees = 0, pendulumVelocityRadians = 0;

// Variáveis para armazenar as últimas posições
long motorLastPositionPulses = 0;
long pendulumLastPositionPulses = 0;
unsigned long lastTime = 0; // Para calcular o intervalo de tempo (Δt)

void setup() {
    Serial.begin(9600);
    lastTime = millis(); // Inicializa o tempo
}

void loop() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Intervalo de tempo em segundos

    // Leitura da posição atual dos encoders (em pulsos)
    long motorPositionPulses = motorEncoder.read();
    long pendulumPositionPulses = pendulumEncoder.read();

    // Converte as posições para graus e radianos
    motorPositionDegrees = motorPositionPulses / pulsesPerDegreeMotor;
    motorPositionRadians = motorPositionPulses / pulsesPerRadianMotor;
    pendulumPositionDegrees = pendulumPositionPulses / pulsesPerDegreePendulum;
    pendulumPositionRadians = pendulumPositionPulses / pulsesPerRadianPendulum;

    // Calcula as velocidades em graus/s e rad/s para o motor
    motorVelocityDegrees = (motorPositionDegrees - (motorLastPositionPulses / pulsesPerDegreeMotor)) / deltaTime;
    motorVelocityRadians = (motorPositionRadians - (motorLastPositionPulses / pulsesPerRadianMotor)) / deltaTime;

    // Calcula as velocidades em graus/s e rad/s para o pêndulo
    pendulumVelocityDegrees = (pendulumPositionDegrees - (pendulumLastPositionPulses / pulsesPerDegreePendulum)) / deltaTime;
    pendulumVelocityRadians = (pendulumPositionRadians - (pendulumLastPositionPulses / pulsesPerRadianPendulum)) / deltaTime;

    // Atualiza as últimas posições e tempo para a próxima leitura
    motorLastPositionPulses = motorPositionPulses;
    pendulumLastPositionPulses = pendulumPositionPulses;
    lastTime = currentTime;

    // Exibe os dados no Serial Monitor no formato solicitado
    Serial.print("graus: (");
    Serial.print(motorPositionDegrees);
    Serial.print(", ");
    Serial.print(motorVelocityDegrees);
    Serial.print(", ");
    Serial.print(pendulumPositionDegrees);
    Serial.print(", ");
    Serial.print(pendulumVelocityDegrees);
    Serial.println(")");

    Serial.print("rad: (");
    Serial.print(motorPositionRadians);
    Serial.print(", ");
    Serial.print(motorVelocityRadians);
    Serial.print(", ");
    Serial.print(pendulumPositionRadians);
    Serial.print(", ");
    Serial.print(pendulumVelocityRadians);
    Serial.println(")");

    Serial.println(); // Linha em branco para separar cada leitura

    // Pequeno atraso para evitar excesso de dados no Serial Monitor
    delay(100);
}
