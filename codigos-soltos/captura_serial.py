"""
captura_serial.py
─────────────────────────────────────────────────────────────
Captura os dados enviados pelo Arduino durante o ensaio em
malha aberta do Pêndulo de Furuta e salva em CSV.

Uso:
    python captura_serial.py [PORTA] [ARQUIVO_SAIDA]

Exemplos:
    Windows:  python captura_serial.py COM3 dados_ensaio.csv
    Linux:    python captura_serial.py /dev/ttyUSB0 dados_ensaio.csv

Dependências:
    pip install pyserial

Formato esperado do Arduino (linhas de dados):
    tempo_ms,passo,pwm,alpha_rad,alphaDot_rad_s
    1020,0,127,0.002613,0.1308
    ...
Linhas começando com '#' são comentários e ficam no log, mas
não vão para o CSV de dados.
─────────────────────────────────────────────────────────────
"""

import sys
import serial
import csv
import time
from datetime import datetime

# ── Configurações ─────────────────────────────────────────────
BAUD_RATE    = 115200
TIMEOUT_S    = 2.0          # s sem dados → considera encerrado
DEFAULT_PORT = "COM3"       # altere para /dev/ttyUSB0 no Linux

# ── Argumentos ────────────────────────────────────────────────
porta  = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
saida  = sys.argv[2] if len(sys.argv) > 2 else "dados_ensaio.csv"

print(f"[INFO] Porta serial : {porta}")
print(f"[INFO] Arquivo saída: {saida}")
print(f"[INFO] Aguardando conexão...")

# ── Conexão ───────────────────────────────────────────────────
try:
    ser = serial.Serial(porta, BAUD_RATE, timeout=1)
except serial.SerialException as e:
    print(f"[ERRO] Não foi possível abrir {porta}: {e}")
    sys.exit(1)

time.sleep(2)   # aguarda o Arduino resetar após abertura da serial

print(f"[INFO] Conectado em {porta} @ {BAUD_RATE} baud")
print("[INFO] Aguardando início do ensaio no Arduino...")
print("       (envie qualquer tecla pela Serial Monitor do Arduino IDE)")
print("       Ou pressione Ctrl+C aqui para encerrar a qualquer momento.\n")

# ── Captura ───────────────────────────────────────────────────
HEADER = ["tempo_ms", "passo", "pwm", "alpha_rad", "alphaDot_rad_s"]
linhas_dados = []
linhas_log   = []

try:
    with open(saida, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)

        ultimo_dado = time.time()
        ensaio_iniciado = False

        while True:
            linha_raw = ser.readline()
            if not linha_raw:
                # Timeout de leitura da serial
                if ensaio_iniciado and (time.time() - ultimo_dado) > TIMEOUT_S:
                    print("\n[INFO] Sem dados por 2 s. Encerrando captura.")
                    break
                continue

            linha = linha_raw.decode("utf-8", errors="replace").strip()

            if not linha:
                continue

            # Linhas de comentário / log
            if linha.startswith("#"):
                print(f"[ARDUINO] {linha}")
                linhas_log.append(linha)
                if "INÍCIO" in linha or "INICIO" in linha:
                    ensaio_iniciado = True
                if "FIM DO ENSAIO" in linha:
                    print("[INFO] Sinal de fim recebido. Encerrando captura.")
                    break
                continue

            # Linha de cabeçalho enviada pelo Arduino — ignorar
            if linha.startswith("tempo_ms"):
                continue

            # Linha de dados
            partes = linha.split(",")
            if len(partes) == 5:
                try:
                    tempo_ms     = int(partes[0])
                    passo        = int(partes[1])
                    pwm          = int(partes[2])
                    alpha_rad    = float(partes[3])
                    alpha_dot    = float(partes[4])

                    linha_csv = [tempo_ms, passo, pwm, alpha_rad, alpha_dot]
                    writer.writerow(linha_csv)
                    linhas_dados.append(linha_csv)
                    ultimo_dado = time.time()

                    # Feedback no terminal a cada 50 amostras
                    if len(linhas_dados) % 50 == 0:
                        print(f"  t={tempo_ms:6d} ms | passo={passo} | "
                              f"pwm={pwm:3d} | α={alpha_rad:+.4f} rad | "
                              f"α̇={alpha_dot:+.3f} rad/s")
                except ValueError:
                    print(f"[AVISO] Linha inválida ignorada: {linha}")

except KeyboardInterrupt:
    print("\n[INFO] Captura interrompida pelo usuário.")

finally:
    ser.close()
    print(f"\n[INFO] Serial fechada.")
    print(f"[INFO] {len(linhas_dados)} amostras salvas em '{saida}'")
    if linhas_dados:
        t0 = linhas_dados[0][0]
        tf = linhas_dados[-1][0]
        print(f"[INFO] Duração do ensaio: {(tf - t0)/1000:.1f} s")
