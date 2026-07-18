

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


/* --------------- SINTONIA  ------ */
const float KC    = 90.0f; // 57.31f <- valor da sintonia LoopShaping/ITAE
const float TI    = 1000.0f; //0.1015 <- valor da sintonia LoopShaping/ITAE
const float TD    = 0.1f; //0.4777 <- valor da sintonia LoopShaping/ITAE
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
/* ------------- ZONA MORTA (Amansada para a bancada) -------- */
// Cortamos o salto inicial pela metade para evitar o "coice" do atrito dinâmico.
const float DZ_POS = 110.0f;  // Era 130 mas com a inercia inicial vencida, o motor trepida. Reduzido para 110.
const float DZ_NEG = 70.0f;  // Era 75 mas com a inercia inicial vencida, o motor trepida. Reduzido para 70.
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
/* --------------- VARIAVEIS DE TELEMETRIA ------------- */
volatile float g_P   = 0.0f;
volatile float g_I   = 0.0f;
volatile float g_D   = 0.0f;
volatile float g_uns = 0.0f; // Esforco antes da zona morta
volatile float g_e   = 0.0f; // Erro

volatile bool  flag_ctrl = false;
volatile float g_alpha   = 0.0f;
volatile float g_u       = 0.0f;


/* ============================================================================
 *  pid_2dof_discreto()  --  chamada EXATAMENTE a cada Ts
 * ========================================================================== */
float pid_2dof_discreto(float r, float y) {

  float e_bruto = r - y; 
  float tol_rad = 2.0f * PI / 180.0f;   /* Ponto morto de +/- 2 graus em radianos */
  
  float e, P, vD;

  /* --- PONTO MORTO DO SENSOR (Relaxa o PID na faixa permitida) --- */
  if (fabs(e_bruto) <= tol_rad) {
    e  = 0.0f;
    P  = 0.0f;
    vD = 0.0f;
    est_I  = 0.0f;    /* Esvaziletamente */
    est_ud = 0.0f;    /* Zera a memória do derivativo para evitar trancos ao sair */
  } else {
    e  = e_bruto ;//a o integrador para o motor desligar comp
    P  = KC * (B_W * r - y);
    vD = C_W * r - y;
  }

  /* --- DERIVATIVO: Euler backward ------------------------------------------ */
  est_ud = AD * est_ud + BD * (vD - est_v_ant);

/* --- INTEGRAL: Tustin / trapezio ----------------------------------------- */
  float I_cand = est_I + KI * (e + est_e_ant);

  float u_ns = P + I_cand + est_ud;               /* comando antes da zona morta */

  /* --- COMPENSACAO INVERSA DA ZONA MORTA COM BANDA MORTA ------------------- */
  float u_cmd;
  
  // BANDA MORTA: Abaixo desse valor de PWM, ignoramos (filtra ruido 1 pulso do encoder)
  // Experimente com 2.0f. Se tremer muito, suba para 3.0f ou 4.0f.
  float deadband = 2.0f; 

  if (u_ns > deadband) {
    // Mapeia u_ns de [deadband ... U_MAX] para [DZ_POS ... U_MAX]
    u_cmd = DZ_POS + (u_ns - deadband) * ((U_MAX - DZ_POS) / (U_MAX - deadband));
  } 
  else if (u_ns < -deadband) {
    // Mapeia u_ns de [-deadband ... -U_MAX] para [-DZ_NEG ... -U_MAX]
    u_cmd = -DZ_NEG + (u_ns + deadband) * ((U_MAX - DZ_NEG) / (U_MAX - deadband));
  } 
  else {
    u_cmd = 0.0f;
  }
  /* --- SATURACAO FISICA DO PWM -------------------------------------------- */
  float u = u_cmd;
  if (u >  U_MAX) u =  U_MAX;
  if (u < -U_MAX) u = -U_MAX;

  /* --- ANTI-WINDUP POR CLAMPING ------------------------------------------- */
  bool saturou       = (u != u_cmd);
  bool mesmo_sentido = ((e > 0.0f) == (u_ns > 0.0f));
  if (!(saturou && mesmo_sentido)) {
    est_I = I_cand;                               
  }                                               

  /* --- ATUALIZACAO DE ESTADOS E TELEMETRIA -------------------------------- */
  est_e_ant = e;
  est_v_ant = vD;
  
  // Salva para imprimir depois
  g_e   = e;
  g_P   = P;
  g_I   = est_I;
  g_D   = est_ud;
  g_uns = u_ns;
  
  return u;
}

/* --------------------- ATUACAO ----------------------- */
void aciona_motor(float u) {
  int pwm = (int)(fabs(u) + 0.5f);
  if (pwm > 255) pwm = 255;
  
  /* --- INVERSAO DE SINAL FEITA AQUI --- */
  if (u < 0.0f) {                    // Antes era (u > 0.0f)
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (u > 0.0f) {             // Antes era (u < 0.0f)
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
  
  /* --- CORRECAO DO GIRO (Limita entre -180 e +180 graus) --- */
  while (alpha >  PI) alpha -= 2.0f * PI;
  while (alpha < -PI) alpha += 2.0f * PI;

  float r = REF_DEG * PI / 180.0f;                              /* rad */

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

  Serial.println(F("tempo,alpha,erro,P,I,D,u_ns,u_final"));
}

/* ------------------------ LOOP ----------------------- */
void loop() {
  if (flag_ctrl) {
    flag_ctrl = false;
    
    // Imprime no formato CSV
    Serial.print(millis());    Serial.print(',');
    Serial.print(g_alpha, 5);  Serial.print(',');
    Serial.print(g_e, 5);      Serial.print(',');
    Serial.print(g_P, 2);      Serial.print(',');
    Serial.print(g_I, 2);      Serial.print(',');
    Serial.print(g_D, 2);      Serial.print(',');
    Serial.print(g_uns, 2);    Serial.print(',');
    Serial.println(g_u, 1);
  }
}