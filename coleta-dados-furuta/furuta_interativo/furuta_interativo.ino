/*
 * ============================================================
 *  PÊNDULO DE FURUTA — MODO INTERATIVO (tempo real)
 *  Entrega 2 — Introdução a Sistemas de Controle — UFMT/FAENG
 * ============================================================
 *
 *  DIFERENÇA PARA O SKETCH ANTERIOR:
 *    Este sketch NÃO tem uma sequência fixa de degraus. Ele fica
 *    sempre lendo a porta serial e aceita comandos enviados pelo
 *    PC (pelo app monitor_tempo_real.py). Assim você escolhe o
 *    tamanho do degrau e a hora de aplicá-lo, e vê o gráfico ao vivo.
 *
 *  PROTOCOLO DE COMANDOS (uma linha por comando, terminada em \n):
 *    P<valor>   → aplica PWM = <valor>  (-255 a 255)  ex: "P150", "P-120"
 *    Z          → zera a contagem do encoder (define alpha = 0)
 *    B          → freio (PWM = 0)
 *    R          → reset: para o motor e zera o encoder
 *
 *  SAÍDA SERIAL (CSV contínuo, sempre que o ensaio está rodando):
 *    tempo_ms , pwm , alpha_rad
 *
 *  A transmissão começa automaticamente no boot (sem esperar 'S').
 *  Isso facilita o streaming ao vivo.
 *
 *  PINAGEM (igual ao sketch de ensaio):
 *    Encoder pêndulo — Canal A : pino 20  (verde)
 *                      Canal B : pino 21  (branco)
 *    Ponte H IN1   : pino 8   (roxo)
 *    Ponte H IN2   : pino 9   (cinza)
 *    Ponte H ENA   : pino 10
 *
 *  DEPENDÊNCIA: biblioteca "Encoder" (Paul Stoffregen)
 * ============================================================
 */

#include <Encoder.h>

// ── Pinos ─────────────────────────────────────────────────────
const int ENA       = 10;
const int IN1       = 8;
const int IN2       = 9;
const int PIN_ENC_A = 20;
const int PIN_ENC_B = 21;

// ── Encoder ───────────────────────────────────────────────────
Encoder encoderPendulo(PIN_ENC_A, PIN_ENC_B);
const float PPR = 2400.0f;   // pulsos/revolução em quadratura x4

// ── Amostragem ────────────────────────────────────────────────
const unsigned long Ts_ms = 20;   // 50 Hz

// ── Estado ────────────────────────────────────────────────────
unsigned long t_ultimo = 0;
int           pwm_atual = 0;       // PWM aplicado no momento

// ── Atuação ───────────────────────────────────────────────────
void aplicarPWM(int pwm_val) {
  // Satura entre -255 e 255
  if (pwm_val > 255)  pwm_val = 255;
  if (pwm_val < -255) pwm_val = -255;
  pwm_atual = pwm_val;

  if (pwm_val > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwm_val);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

// ── Processa um comando recebido pela serial ──────────────────
void processarComando(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = cmd.charAt(0);

  switch (c) {
    case 'P':   // P<valor> → aplica PWM
    case 'p': {
      int valor = cmd.substring(1).toInt();
      aplicarPWM(valor);
      Serial.print("# CMD PWM=");
      Serial.println(valor);
      break;
    }
    case 'Z':   // zera encoder
    case 'z':
      encoderPendulo.write(0);
      Serial.println("# CMD ZERO ENCODER");
      break;
    case 'B':   // freio
    case 'b':
      aplicarPWM(0);
      Serial.println("# CMD FREIO");
      break;
    case 'R':   // reset total
    case 'r':
      aplicarPWM(0);
      encoderPendulo.write(0);
      Serial.println("# CMD RESET");
      break;
    default:
      Serial.print("# CMD DESCONHECIDO: ");
      Serial.println(cmd);
      break;
  }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);   // baud mais alto para streaming ao vivo

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  aplicarPWM(0);
  encoderPendulo.write(0);

  Serial.println("# FURUTA MODO INTERATIVO PRONTO");
  Serial.println("tempo_ms,pwm,alpha_rad");   // cabeçalho CSV

  t_ultimo = millis();
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  // ── Lê comandos da serial (não bloqueante) ────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processarComando(cmd);
  }

  // ── Amostragem periódica e envio CSV ──────────────────────
  unsigned long agora = millis();
  if ((agora - t_ultimo) >= Ts_ms) {
    t_ultimo = agora;

    long  pulsos   = encoderPendulo.read();
    float alphaRad = 2.0f * PI * (float)pulsos / PPR;

    Serial.print(agora);
    Serial.print(",");
    Serial.print(pwm_atual);
    Serial.print(",");
    Serial.println(alphaRad, 6);
  }
}
