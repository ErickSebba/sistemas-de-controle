#include <Encoder.h>
#include <TimerOne.h>

// Pinos da Ponte H (motor)
const int pinIN1 = 2;
const int pinIN2 = 3;
const int pinPWM = 5;

// Pinos do Encoder do Motor e do Pêndulo
const int motorEncoderPinA = 18;   // Fio Amarelo
const int motorEncoderPinB = 19;   // Fio Verde
const int pendulumEncoderPinA = 20; // Fio Branco
const int pendulumEncoderPinB = 21; // Fio Verde

Encoder motorEncoder(motorEncoderPinA, motorEncoderPinB);
Encoder pendulumEncoder(pendulumEncoderPinA, pendulumEncoderPinB);

// Matrizes de controle
const float K[4] = {-0.2307,   -0.1698,    3.7649,    0.5462};

// Variáveis globais para armazenar o estado do sistema
volatile float motorPosition = 0, motorLastPosition = 0;
volatile float pendulumPosition = 0, pendulumLastPosition = 0;
volatile float motorVelocity = 0, pendulumVelocity = 0;

// Intervalo de controle em microssegundos
const unsigned long controlInterval = 2000; // 2 ms (500 Hz)

// PWM mínimo para iniciar o motor (zona morta)
const int minPWM = 100;

void setup() {
    Serial.begin(9600);

    // Configura pinos da ponte H
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);
    pinMode(pinPWM, OUTPUT);

    // Inicializa o motor parado
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
    analogWrite(pinPWM, 0);

    // Configura o Timer para interrupções
    Timer1.initialize(controlInterval);
    Timer1.attachInterrupt(controlLoop);

    Serial.println("Sistema inicializado com interrupções.");
}

void controlLoop() {
    // Leitura das posições dos encoders
    motorPosition = motorEncoder.read() % 360; // Limita a posição a 0-359 graus
    pendulumPosition = pendulumEncoder.read() % 360;

    // Cálculo das velocidades
    motorVelocity = (motorPosition - motorLastPosition) / (controlInterval / 1e6);
    pendulumVelocity = (pendulumPosition - pendulumLastPosition) / (controlInterval / 1e6);

    // Atualiza as últimas posições
    motorLastPosition = motorPosition;
    pendulumLastPosition = pendulumPosition;

    // Vetor de estados [theta, dot(theta), alpha, dot(alpha)]
    float x[4] = {motorPosition, motorVelocity, pendulumPosition, pendulumVelocity};

    float maxTau = 200; // Saturação dinâmica

    // Calcula a entrada de controle (torque) usando realimentação de estados
    float tau = constrain(-(K[0] * x[0] + K[1] * x[1] + K[2] * x[2] + K[3] * x[3]), -maxTau, maxTau);

    // Limita o valor de PWM para controlar a agressividade do motor
    int pwmValue = constrain(abs(tau), 0, 255);

    // Aplica zona morta: se o valor de pwmValue estiver abaixo do mínimo necessário, força para minPWM
    if (pwmValue > 0 && pwmValue < minPWM) {
        pwmValue = minPWM;
    }

    if (tau > 0) {
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);
    } else {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);
    }

    analogWrite(pinPWM, pwmValue);
}

void loop() {
    // Exibe os dados no Serial Monitor periodicamente
    static unsigned long lastPrint = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastPrint >= 100) { // A cada 100 ms
        lastPrint = currentMillis;

        Serial.print("Motor Position: ");
        Serial.print(motorPosition);
        Serial.print(" | Motor Velocity: ");
        Serial.print(motorVelocity);
        Serial.print(" | Pendulum Position: ");
        Serial.print(pendulumPosition);
        Serial.print(" | Pendulum Velocity: ");
        Serial.print(pendulumVelocity);
        Serial.print(" | Torque (Tau): ");
        Serial.println(-(K[0] * motorPosition + K[1] * motorVelocity + K[2] * pendulumPosition + K[3] * pendulumVelocity));
    }
}
