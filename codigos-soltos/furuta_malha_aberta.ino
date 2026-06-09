/*
 * ============================================================
 *  PÊNDULO DE FURUTA — ENSAIO EM MALHA ABERTA (Entrega 2)
 *  Disciplina: Introdução a Sistemas de Controle — UFMT/FAENG
 *  Configuração: posição inferior (equilíbrio estável)
 * ============================================================
 *
 *  DESCRIÇÃO:
 *    Este sketch executa a sequência de ensaio em malha aberta
 *    definida pelo professor:
 *      Passo 0: U_op        (valor base, ~50% PWM)
 *      Passo 1: U_op + 25%  (degrau positivo)
 *      Passo 2: U_op        (retorno à base)
 *      Passo 3: U_op - 25%  (degrau negativo)
 *      Passo 4: U_op        (retorno final)
 *
 *    A cada ciclo de amostragem (Ts = 20 ms) envia via Serial:
 *      tempo_ms,passo,pwm,alpha_rad,alphaDot_rad_s
 *
 *  PINAGEM (conforme Tabela 4 do relatório):
 *    Encoder pêndulo — Canal A: pino 20 (INT1)
 *                      Canal B: pino 21 (INT0)
 *    Ponte H IN1: pino 2
 *    Ponte H IN2: pino 3
 *    Ponte H PWM: pino 5
 *
 *  DEPENDÊNCIAS:
 *    - Biblioteca "Encoder" (Paul Stoffregen) — instale pela IDE
 *    - Biblioteca "TimerOne"                  — instale pela IDE
 * ============================================================
 */

#include <Encoder.h>
#include <TimerOne.h>

// ── Pinos ────────────────────────────────────────────────────
const int PIN_ENC_A = 20;
const int PIN_ENC_B = 21;
const int PIN_IN1   = 8;   // IN1 confirmado no teste
const int PIN_IN2   = 9;   // IN2 confirmado no teste
const int PIN_PWM   = 10;  // ENA (PWM) confirmado no teste

// ── Encoder ──────────────────────────────────────────────────
Encoder pendEncoder(PIN_ENC_A, PIN_ENC_B);
const float PPR = 2400.0f;   // pulsos/revolução em quadratura x4

// ── Parâmetros do ensaio ──────────────────────────────────────
// U_op = 50% de 255 ≈ 127 PWM
// Ajuste U_OP se necessário após verificar ponto de operação real
const int U_OP        = 127;   // PWM base (~50%)
const int U_DEGRAU    = 64;    // 25% de 255 ≈ 64 PWM

// Duração de cada passo em milissegundos
const unsigned long T_PASSO_MS = 3000UL;  // 3 s por passo

// ── Temporização ─────────────────────────────────────────────
const long Ts_us = 20000L;   // 20 ms em microssegundos
volatile bool novoCiclo = false;

// ── Variáveis de estado ───────────────────────────────────────
float alphaRad_prev  = 0.0f;
unsigned long t_inicio = 0;
int passoAtual = -1;   // -1 = aguardando início

// Tabela de passos: {pwm, sentido (1=frente, -1=ré, 0=parado)}
// Sentido é controlado pelos pinos IN1/IN2
struct Passo {
  int    pwm;
  int    sentido;  // 1 ou -1
  String nome;
};

Passo passos[] = {
  { U_OP,              1,  "U_op"       },   // passo 0
  { U_OP + U_DEGRAU,   1,  "U_op+25%"   },   // passo 1
  { U_OP,              1,  "U_op"       },   // passo 2
  { U_OP - U_DEGRAU,   1,  "U_op-25%"   },   // passo 3
  { U_OP,              1,  "U_op"       },   // passo 4
};
const int N_PASSOS = 5;

// ── ISR do temporizador ───────────────────────────────────────
void isrTimer() {
  novoCiclo = true;
}

// ── Funções de atuação ────────────────────────────────────────
void aplicarPWM(int pwm, int sentido) {
  // Satura entre 0 e 255
  pwm = constrain(pwm, 0, 255);

  if (pwm == 0) {
    // Freio: ambos os pinos em LOW
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, 0);
    return;
  }

  if (sentido >= 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  analogWrite(PIN_PWM, pwm);
}

void pararMotor() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_PWM, 0);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pararMotor();

  // Aguarda comando do usuário para iniciar (evita início acidental)
  Serial.println("# PÊNDULO DE FURUTA — ENSAIO EM MALHA ABERTA");
  Serial.println("# Posicione o pendulo na posicao inferior (0 graus).");
  Serial.println("# Envie qualquer caractere para iniciar o ensaio.");
  Serial.println("# Formato de dados: tempo_ms,passo,pwm,alpha_rad,alphaDot_rad_s");

  while (!Serial.available()) { /* aguarda */ }
  while (Serial.available()) Serial.read();  // limpa buffer

  // Zera encoder na posição atual (referência = posição inferior)
  pendEncoder.write(0);
  alphaRad_prev = 0.0f;

  t_inicio = millis();
  passoAtual = 0;

  // Inicia timer
  Timer1.initialize(Ts_us);
  Timer1.attachInterrupt(isrTimer);

  Serial.println("# INÍCIO DO ENSAIO");
  Serial.println("tempo_ms,passo,pwm,alpha_rad,alphaDot_rad_s");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  if (!novoCiclo) return;
  novoCiclo = false;

  // ── Gerenciamento de passos ──────────────────────────────────
  unsigned long agora = millis();
  int passoCalculado = (int)((agora - t_inicio) / T_PASSO_MS);

  if (passoCalculado >= N_PASSOS) {
    // Ensaio concluído
    pararMotor();
    Serial.println("# FIM DO ENSAIO");
    Timer1.detachInterrupt();
    while (true) { /* trava aqui */ }
  }

  if (passoCalculado != passoAtual) {
    passoAtual = passoCalculado;
    Serial.print("# Iniciando passo ");
    Serial.print(passoAtual);
    Serial.print(" — ");
    Serial.println(passos[passoAtual].nome);
  }

  // ── Atuação ──────────────────────────────────────────────────
  aplicarPWM(passos[passoAtual].pwm, passos[passoAtual].sentido);

  // ── Medição ──────────────────────────────────────────────────
  long pulsos    = pendEncoder.read();
  float alphaRad = 2.0f * PI * (float)pulsos / PPR;

  // Derivada discreta (rad/s)
  float Ts_s       = (float)Ts_us / 1e6f;
  float alphaDot   = (alphaRad - alphaRad_prev) / Ts_s;
  alphaRad_prev    = alphaRad;

  // ── Envio serial (CSV) ────────────────────────────────────────
  Serial.print(agora);            Serial.print(",");
  Serial.print(passoAtual);       Serial.print(",");
  Serial.print(passos[passoAtual].pwm); Serial.print(",");
  Serial.print(alphaRad, 6);      Serial.print(",");
  Serial.println(alphaDot, 4);
}
