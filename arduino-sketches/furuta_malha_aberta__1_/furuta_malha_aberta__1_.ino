
/*
 * ============================================================
 *  PÊNDULO DE FURUTA — ENSAIO EM MALHA ABERTA (Entrega 2)
 *  Disciplina: Introdução a Sistemas de Controle — UFMT/FAENG
 *  Configuração: posição inferior (equilíbrio estável)
 * ============================================================
 *
 *  PROTEÇÃO ANTI-ENROLAMENTO DE CABO
 *    O braço não tem encoder nem fim-de-curso, então a posição
 *    angular do braço é estimada pelo tempo de giro acumulado
 *    em cada sentido.
 *
 *    LIMITE_GIRO_MS define o tempo máximo contínuo permitido
 *    num mesmo sentido (padrão: 1500 ms ≈ ~180° dependendo do
 *    motor). Quando atingido, o sentido é INVERTIDO antes de
 *    continuar o ensaio.
 *
 *    Isso garante que o braço sempre oscila de volta ao centro
 *    e nunca acumula voltas suficientes para enrolar o cabo.
 *
 *  SEQUÊNCIA DE ENSAIO (definida pelo professor):
 *    Passo 0: U_op          (~50% PWM, estado base)
 *    Passo 1: U_op + 25%    (degrau positivo)
 *    Passo 2: U_op          (retorno à base)
 *    Passo 3: U_op - 25%    (degrau negativo)
 *    Passo 4: U_op          (retorno final)
 *
 *  SAÍDA SERIAL (CSV a cada Ts = 20 ms):
 *    tempo_ms, passo, pwm, sentido, alpha_rad, alphaDot_rad_s
 *
 *  PINAGEM CONFIRMADA NOS TESTES:
 *    Encoder pêndulo — Canal A : pino 20
 *                      Canal B : pino 21
 *    Ponte H  IN1    : pino 8
 *             IN2    : pino 9
 *             ENA    : pino 10
 *
 *  DEPENDÊNCIAS:
 *    Biblioteca "Encoder"  (Paul Stoffregen) — Gerenciador IDE
 *    Biblioteca "TimerOne"                   — Gerenciador IDE
 * ============================================================
 */

#include <Encoder.h>
#include <TimerOne.h>

// ── Pinos ────────────────────────────────────────────────────
const int PIN_ENC_A = 20;
const int PIN_ENC_B = 21;
const int PIN_IN1   = 8;   // confirmado no teste
const int PIN_IN2   = 9;   // confirmado no teste
const int PIN_PWM   = 10;  // ENA — confirmado no teste

// ── Encoder do pêndulo ────────────────────────────────────────
Encoder pendEncoder(PIN_ENC_A, PIN_ENC_B);
const float PPR = 2400.0f;   // pulsos/rev com quadratura x4

// ── Parâmetros do ensaio ──────────────────────────────────────
const int U_OP     = 127;   // ~50% de 255 (ajuste se necessário)
const int U_DEGRAU =  64;   // ~25% de 255

const unsigned long T_PASSO_MS = 3000UL;  // duração de cada passo

// ── Proteção anti-enrolamento ─────────────────────────────────
// Tempo máximo permitido girando no mesmo sentido.
// Aumente se o braço for curto / motor lento.
// Diminua se o cabo for curto ou a montagem tiver pouca folga.
const unsigned long LIMITE_GIRO_MS = 1500UL;  // 1.5 s por sentido

int sentidoAtual = 1;                  // +1 = frente, -1 = ré
unsigned long t_ultimo_inversao = 0;   // millis() da última inversão

// ── Temporização (ISR) ────────────────────────────────────────
const long Ts_us = 20000L;            // 20 ms
volatile bool novoCiclo = false;
void isrTimer() { novoCiclo = true; }

// ── Variáveis de estado ───────────────────────────────────────
float alphaRad_prev = 0.0f;
unsigned long t_inicio = 0;
int passoAtual = -1;

// ── Tabela de passos ──────────────────────────────────────────
struct Passo { int pwm; const char* nome; };
Passo passos[] = {
  { U_OP,            "U_op"     },   // 0
  { U_OP + U_DEGRAU, "U_op+25%" },   // 1
  { U_OP,            "U_op"     },   // 2
  { U_OP - U_DEGRAU, "U_op-25%" },   // 3
  { U_OP,            "U_op"     },   // 4
};
const int N_PASSOS = 5;

// ── Funções de atuação ────────────────────────────────────────
void aplicarPWM(int pwm, int sentido) {
  pwm = constrain(pwm, 0, 255);
  if (pwm == 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, 0);
    return;
  }
  if (sentido > 0) {
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

// ── Proteção anti-enrolamento ─────────────────────────────────
// Chamada a cada ciclo. Inverte sentidoAtual se o braço já
// girou tempo demais no mesmo sentido.
void verificarLimiteGiro(unsigned long agora) {
  if ((agora - t_ultimo_inversao) >= LIMITE_GIRO_MS) {
    sentidoAtual     = -sentidoAtual;   // inverte
    t_ultimo_inversao = agora;
    Serial.print("# [ANTIROLL] Sentido invertido para ");
    Serial.println(sentidoAtual > 0 ? "FRENTE" : "RE");
  }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pararMotor();

  Serial.println("# PENDULO DE FURUTA — ENSAIO EM MALHA ABERTA");
  Serial.println("# Posicione o pendulo na posicao inferior.");
  Serial.println("# ANTI-ENROLAMENTO ATIVO: sentido inverte a cada 1.5 s.");
  Serial.println("# Envie qualquer caractere para iniciar.");
  Serial.println("# Formato: tempo_ms,passo,pwm,sentido,alpha_rad,alphaDot_rad_s");

  while (!Serial.available()) {}
  while (Serial.available()) Serial.read();

  // Pull-ups necessários para o encoder NPN (coletor aberto)
  // conforme seção 4.2 do relatório
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  pendEncoder.write(0);
  alphaRad_prev     = 0.0f;
  t_inicio          = millis();
  t_ultimo_inversao = t_inicio;
  passoAtual        = 0;

  Timer1.initialize(Ts_us);
  Timer1.attachInterrupt(isrTimer);

  Serial.println("# INICIO DO ENSAIO");
  Serial.println("tempo_ms,passo,pwm,sentido,alpha_rad,alphaDot_rad_s");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  if (!novoCiclo) return;
  novoCiclo = false;

  unsigned long agora = millis();

  // ── Gerencia passos ───────────────────────────────────────
  int passoCalculado = (int)((agora - t_inicio) / T_PASSO_MS);

  if (passoCalculado >= N_PASSOS) {
    pararMotor();
    Serial.println("# FIM DO ENSAIO");
    Timer1.detachInterrupt();
    while (true) {}
  }

  if (passoCalculado != passoAtual) {
    passoAtual = passoCalculado;
    Serial.print("# Passo ");
    Serial.print(passoAtual);
    Serial.print(" — ");
    Serial.println(passos[passoAtual].nome);
  }

  // ── Anti-enrolamento: verifica e inverte se necessário ────
  verificarLimiteGiro(agora);

  // ── Atuação com sentido protegido ─────────────────────────
  aplicarPWM(passos[passoAtual].pwm, sentidoAtual);

  // ── Medição do encoder do pêndulo ─────────────────────────
  long  pulsos   = pendEncoder.read();
  float alphaRad = 2.0f * PI * (float)pulsos / PPR;
  float Ts_s     = (float)Ts_us / 1e6f;
  float alphaDot = (alphaRad - alphaRad_prev) / Ts_s;
  alphaRad_prev  = alphaRad;

  // ── Saída CSV ─────────────────────────────────────────────
  Serial.print(agora);                      Serial.print(",");
  Serial.print(passoAtual);                 Serial.print(",");
  Serial.print(passos[passoAtual].pwm);     Serial.print(",");
  Serial.print(sentidoAtual);               Serial.print(",");
  Serial.print(alphaRad, 6);                Serial.print(",");
  Serial.println(alphaDot, 4);
}
