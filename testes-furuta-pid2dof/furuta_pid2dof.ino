/* ============================================================================
 *  furuta_pid2dof.ino
 *  Controle PID 2-DOF DISCRETO do Pendulo de Furuta (posicao INFERIOR)
 *
 *  Disciplina: Introducao a Sistemas de Controle - 2026/1  |  UFMT/FAENG
 *  Prof. Daniel Miranda Cruz
 *  Grupo: Daniel M. Zamarioli, Erick G. B. Sebba, Fabio C. Barbosa Jr.
 *
 *  CENARIO: REGULATORIO. Mantem alpha = 0 (pendulo pendurado) e rejeita
 *  perturbacoes. Seguimento de referencia disponivel (ver REF_DEG).
 *
 *  ---------------------------------------------------------------------------
 *  LEI DE CONTROLE (forma ISA de 3 termos, 2 graus de liberdade)
 *
 *      u = Kc*(b*r - y)  +  I  +  D
 *
 *    P : Kc*(b*r - y)          -> peso b na referencia (2-DOF)
 *    I : TUSTIN (trapezio)     -> I(k) = I(k-1) + Ki*( e(k) + e(k-1) )
 *                                 Ki = Kc*Ts/(2*Ti)
 *    D : EULER (backward)      -> ud(k) = aD*ud(k-1) + bD*( vD(k) - vD(k-1) )
 *                                 aD = Tf/(Tf+Ts) ,  bD = Kc*Td/(Tf+Ts)
 *                                 vD = c*r - y  ->  com c=0:  vD = -y
 *
 *  POR QUE DISCRETIZACAO MISTA?  (justificativa completa no relatorio)
 *   - Integral com TUSTIN: metodo de 2a ordem (mais preciso que Euler) e mapeia
 *     o integrador exatamente em z=1  ->  erro nulo em regime garantido.
 *   - Derivativo com EULER: o polo do filtro fica  aD = Tf/(Tf+Ts), que e'
 *     SEMPRE 0 < aD < 1 para QUALQUER Ts. Com Tustin seria
 *     aD = (2Tf-Ts)/(2Tf+Ts), que fica NEGATIVO se Ts > 2*Tf (= 119 ms aqui),
 *     gerando RINGING (oscilacao de periodo 2Ts) no termo derivativo.
 *     Euler torna o firmware robusto a jitter/variacao do periodo de amostragem.
 *
 *  c = 0  ->  derivada so na MEDICAO (evita o "kick" derivativo no degrau de r)
 *  ---------------------------------------------------------------------------
 *
 *  HARDWARE
 *    Arduino Mega 2560
 *    Encoder E38S6G5-600B-G24N : 600 PPR -> 2400 pulsos/volta (quadratura x4)
 *                                pinos 20 e 21 (interrupcao)
 *    Ponte H L298N             : IN1=8, IN2=9, PWM(ENA)=10
 *    Motor CC 12 V
 * ============================================================================ */

#include <TimerOne.h>

/* ----------------------- PINOS ----------------------- */
const uint8_t PIN_ENC_A = 20;
const uint8_t PIN_ENC_B = 21;
const uint8_t PIN_IN1   = 8;
const uint8_t PIN_IN2   = 9;
const uint8_t PIN_PWM   = 10;

/* ----------------------- AMOSTRAGEM ------------------ */
const float    TS      = 0.020f;      // 20 ms  (50 Hz)
const uint32_t TS_US   = 20000UL;

/* --------------- SINTONIA (candidato escolhido) ------ */
/*  Selecionada na Fase 2 (comparacao dos metodos no modelo linear):
 *  unica com margem de fase alta (53 deg), sem overshoot e esforco folgado.  */
const float KC    = 57.31f;
const float TI    = 0.1015f;          // s
const float TD    = 0.4777f;          // s
const float GAMMA = 0.125f;           // 1/8  (filtro derivativo do curso)
const float TF    = GAMMA * TD;       // 0.059713 s
const float B_W   = 0.7f;             // peso b (proporcional na referencia)
const float C_W   = 0.0f;             // peso c = 0 -> derivada so na medicao

/* --------- COEFICIENTES PRE-CALCULADOS (Ts = 20 ms) --- */
const float AD = TF / (TF + TS);            // 0.749098   (EULER)
const float BD = KC * TD / (TF + TS);       // 343.4466   (EULER)
const float KI = KC * TS / (2.0f * TI);     // 5.646305   (TUSTIN)

/* ------------- ZONA MORTA (medida em bancada) -------- */
/*  Sentido + (anti-horario): motor parte em PWM ~120-140  -> adotado 130
 *  Sentido - (horario)     : motor parte em PWM ~ 70-80   -> adotado  75
 *  ASSIMETRICA -> compensacao inversa OBRIGATORIA (sem ela o esforco do
 *  controlador, ~90 PWM, cabe DENTRO da banda morta e o motor quase nao move). */
const float DZ_POS = 130.0f;
const float DZ_NEG = 75.0f;
const float U_MAX  = 255.0f;

/* --------------------- REFERENCIA -------------------- */
const float REF_DEG = 0.0f;           // REGULATORIO: manter o pendulo em 0 grau

/* --------------------- ENCODER ----------------------- */
const float PULSOS_POR_VOLTA = 2400.0f;
volatile long enc_count = 0;
volatile uint8_t enc_ab = 0;

/* Tabela de quadratura (x4). Indice = (antigo<<2)|novo */
const int8_t QTAB[16] = { 0,-1, 1, 0,
                          1, 0, 0,-1,
                         -1, 0, 0, 1,
                          0, 1,-1, 0 };

void isr_encoder() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t novo = (a << 1) | b;
  enc_count += QTAB[(enc_ab << 2) | novo];
  enc_ab = novo;
}

/* --------------- ESTADOS DO CONTROLADOR -------------- */
float est_I     = 0.0f;    // integrador
float est_e_ant = 0.0f;    // erro anterior      (Tustin precisa de e(k-1))
float est_v_ant = 0.0f;    // vD anterior        (Euler precisa de vD(k-1))
float est_ud    = 0.0f;    // termo derivativo filtrado

volatile bool  flag_ctrl = false;
volatile float g_alpha   = 0.0f;
volatile float g_u       = 0.0f;

/* ============================================================================
 *  pid_2dof_discreto()  --  chamada EXATAMENTE a cada Ts
 *  Retorna o PWM ja compensado (zona morta) e saturado.
 * ========================================================================== */
float pid_2dof_discreto(float r, float y) {

  float e  = r - y;                       /* erro (alimenta o integral)        */
  float P  = KC * (B_W * r - y);          /* proporcional com peso b           */
  float vD = C_W * r - y;                 /* c = 0  ->  vD = -y                */

  /* --- DERIVATIVO: Euler backward (aD sempre em (0,1) -> sem ringing) ------ */
  est_ud = AD * est_ud + BD * (vD - est_v_ant);

  /* --- INTEGRAL: Tustin / trapezio ----------------------------------------- */
  float I_cand = est_I + KI * (e + est_e_ant);

  float u_ns = P + I_cand + est_ud;       /* comando antes de saturar          */

  /* --- COMPENSACAO INVERSA DA ZONA MORTA (assimetrica) --------------------- */
  float u_cmd;
  if      (u_ns >  1e-6f) u_cmd = u_ns + DZ_POS;
  else if (u_ns < -1e-6f) u_cmd = u_ns - DZ_NEG;
  else                    u_cmd = 0.0f;

  /* --- SATURACAO FISICA DO PWM -------------------------------------------- */
  float u = u_cmd;
  if (u >  U_MAX) u =  U_MAX;
  if (u < -U_MAX) u = -U_MAX;

  /* --- ANTI-WINDUP POR CLAMPING -------------------------------------------
   * So integra se NAO estiver saturado empurrando o erro mais fundo.        */
  bool saturou       = (u != u_cmd);
  bool mesmo_sentido = ((e > 0.0f) == (u_ns > 0.0f));
  if (!(saturou && mesmo_sentido)) {
    est_I = I_cand;                       /* aceita o novo integrador          */
  }                                       /* senao: congela (nao acumula)      */

  est_e_ant = e;
  est_v_ant = vD;
  return u;
}

/* --------------------- ATUACAO ----------------------- */
void aciona_motor(float u) {
  int pwm = (int)(fabs(u) + 0.5f);
  if (pwm > 255) pwm = 255;
  if (u > 0.0f) {                    /* anti-horario (visto de cima) */
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (u < 0.0f) {             /* horario */
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    pwm = 0;
  }
  analogWrite(PIN_PWM, pwm);
}

/* --------------- LACO DE CONTROLE (ISR) -------------- */
void laco_controle() {
  noInterrupts();
  long pulsos = enc_count;
  interrupts();

  float alpha = 2.0f * PI * (float)pulsos / PULSOS_POR_VOLTA;   /* rad */
  float r     = REF_DEG * PI / 180.0f;                          /* rad */

  float u = pid_2dof_discreto(r, alpha);
  aciona_motor(u);

  g_alpha   = alpha;
  g_u       = u;
  flag_ctrl = true;
}

/* ----------------------- SETUP ----------------------- */
void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);

  enc_ab = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isr_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isr_encoder, CHANGE);

  /* IMPORTANTE: zerar o encoder com o pendulo PARADO na vertical inferior. */
  delay(2000);
  enc_count = 0;
  est_I = est_e_ant = est_v_ant = est_ud = 0.0f;

  Timer1.initialize(TS_US);
  Timer1.attachInterrupt(laco_controle);

  Serial.println(F("tempo_ms,alpha_rad,u_pwm"));
}

/* ------------------------ LOOP ----------------------- */
void loop() {
  if (flag_ctrl) {
    flag_ctrl = false;
    Serial.print(millis());   Serial.print(',');
    Serial.print(g_alpha, 5); Serial.print(',');
    Serial.println(g_u, 1);
  }
}
