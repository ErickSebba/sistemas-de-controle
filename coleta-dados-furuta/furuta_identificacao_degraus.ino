/*
 * ============================================================
 *  PÊNDULO DE FURUTA — IDENTIFICAÇÃO DE SISTEMA (Entrega 2)
 *  Disciplina: Introdução a Sistemas de Controle — UFMT/FAENG
 *  Configuração: posição inferior (equilíbrio estável)
 * ============================================================
 *
 *  OBJETIVO:
 *    Excitar o braço com degraus alternados (ida/volta) e
 *    registrar entrada (PWM) e saída (posição angular do braço)
 *    para calcular a função de transferência G(s) = Alpha(s)/U(s).
 *
 *  SEQUÊNCIA DE EXCITAÇÃO:
 *    Degrau 0: PWM = +150  por 800 ms  → gira p/ frente
 *    Degrau 1: PWM =    0  por 800 ms  → freio (braço desacelera)
 *    Degrau 2: PWM = -150  por 800 ms  → gira p/ trás
 *    Degrau 3: PWM =    0  por 2000 ms → repouso final
 *
 *  SAÍDA SERIAL (CSV, iniciada pelo caractere 'S'):
 *    tempo_ms , pwm , alpha_rad
 *
 *  PINAGEM (conforme Tabela 4 do relatório):
 *    Encoder braço — Canal A : pino 20
 *                    Canal B : pino 21
 *    Ponte H IN1   : pino 8
 *    Ponte H IN2   : pino 9
 *    Ponte H ENA   : pino 10
 *
 *  DEPENDÊNCIAS:
 *    - Biblioteca "Encoder" (Paul Stoffregen) — instale pela IDE
 * ============================================================
 */

#include <Encoder.h>

// ── Pinos ─────────────────────────────────────────────────────
const int ENA       = 10;  // ENA (PWM) confirmado no teste
const int IN1       = 8;   // IN1 confirmado no teste  //roxo
const int IN2       = 9;   // IN2 confirmado no teste  //cinza
const int PIN_ENC_A = 20;  // verde
const int PIN_ENC_B = 21;  // branco

// ── Encoder ───────────────────────────────────────────────────
Encoder encoderPendulo(PIN_ENC_A, PIN_ENC_B);
const float PPR = 2400.0f;  // pulsos/revolução em quadratura x4

// ── Período de amostragem ─────────────────────────────────────
const unsigned long Ts_ms = 20;  // 20 ms → 50 Hz (suficiente para G(s) do braço)

// ── Sequência de degraus ──────────────────────────────────────
// PWM positivo = frente, negativo = trás, zero = freio.
// Ajuste o valor de pwm se o motor não vencer a fricção (suba de 10 em 10).
// Ajuste duracao_ms se o braço ainda estiver acelerando ao fim do degrau.
struct Degrau {
  int           pwm;
  unsigned long duracao_ms;
};

const int N_DEGRAUS = 4;
Degrau sequencia[N_DEGRAUS] = {
  {  150,  800 },   // degrau 0: frente por 800 ms
  {    0,  800 },   // degrau 1: freio  por 800 ms
  { -150,  800 },   // degrau 2: trás   por 800 ms
  {    0, 2000 },   // degrau 3: repouso final (braço para completamente)
};

// ── Variáveis de estado ───────────────────────────────────────
unsigned long t_ultimo        = 0;
unsigned long t_inicio_degrau = 0;
int           degrau_atual    = 0;
bool          ensaio_ativo    = false;
bool          ensaio_encerrado = false;

// ── Funções de atuação ────────────────────────────────────────
void aplicarPWM(int pwm_val) {
  if (pwm_val > 0) {
    // Gira para frente
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm_val);
  }
  else if (pwm_val < 0) {
    // Gira para trás (inverte IN1/IN2; analogWrite só aceita positivo)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwm_val);
  }
  else {
    // Freio: ambos em LOW
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void pararMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pararMotor();

  // Pull-ups internos para o encoder
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  // Aguarda comando do Python (ou monitor serial) para iniciar
  Serial.println("# PÊNDULO DE FURUTA — IDENTIFICAÇÃO DE SISTEMA");
  Serial.println("# Posicione o braco na posicao inferior (referencia = 0 graus).");
  Serial.println("# Envie 'S' para iniciar o ensaio.");

  while (true) {
    if (Serial.available() && Serial.read() == 'S') break;
  }

  // Cabeçalho CSV — apenas as colunas necessárias para identificar G(s)
  Serial.println("tempo_ms,pwm,alpha_rad");

  // Zera encoder e inicia sequência
  encoderPendulo.write(0);
  t_ultimo        = millis();
  t_inicio_degrau = millis();
  degrau_atual    = 0;
  ensaio_ativo    = true;

  aplicarPWM(sequencia[0].pwm);

  Serial.println("# INICIO DO ENSAIO");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {

  // ── Ensaio encerrado: mantém motor parado ─────────────────
  if (ensaio_encerrado) {
    pararMotor();
    return;
  }

  unsigned long agora = millis();

  // ── Gerenciamento de degraus ──────────────────────────────
  if ((agora - t_inicio_degrau) >= sequencia[degrau_atual].duracao_ms) {
    degrau_atual++;

    if (degrau_atual >= N_DEGRAUS) {
      // Sequência concluída
      pararMotor();
      Serial.println("# FIM DO ENSAIO");
      ensaio_encerrado = true;
      return;
    }

    // Aplica o próximo degrau e registra o instante de início
    aplicarPWM(sequencia[degrau_atual].pwm);
    t_inicio_degrau = agora;
  }

  // ── Amostragem periódica ──────────────────────────────────
  if ((agora - t_ultimo) >= Ts_ms) {
    t_ultimo = agora;

    // Leitura do encoder e conversão para radianos
    long  pulsos   = encoderPendulo.read();
    float alphaRad = 2.0f * PI * (float)pulsos / PPR;

    // Envio CSV: tempo | PWM aplicado | posição angular (rad)
    Serial.print(agora);
    Serial.print(",");
    Serial.print(sequencia[degrau_atual].pwm);
    Serial.print(",");
    Serial.println(alphaRad, 6);
  }
}
