"""
============================================================
 coletar_dados.py
 PÊNDULO DE FURUTA — Coleta de dados via Serial
 Entrega 2 — Introdução a Sistemas de Controle — UFMT
============================================================

COMO USAR:
  1. Faça upload do sketch furuta_malha_aberta.ino no Arduino Mega.
  2. Feche o Monitor Serial da Arduino IDE (libera a porta COM).
  3. Execute este script:
       python coletar_dados.py
  4. O script detecta automaticamente a porta COM disponível,
     envia o sinal de início ('S') e salva os dados em CSV.

DEPENDÊNCIAS:
  pip install pyserial

SAÍDA:
  dados_malha_aberta_YYYYMMDD_HHMMSS.csv
  (mesmo diretório do script)
============================================================
"""

import serial
import serial.tools.list_ports
import time
import csv
import sys
from datetime import datetime
from pathlib import Path

# ── Configurações ─────────────────────────────────────────────
BAUD_RATE     = 115200
TIMEOUT_S     = 2           # Timeout de leitura serial (s)
INICIO_CMD    = b'S'        # Comando de início enviado ao Arduino
COMENTARIO    = '#'         # Linhas de log (não são dados)

# ── Detecção automática de porta ──────────────────────────────
def detectar_porta() -> str:
    """Retorna a primeira porta COM com Arduino ou pede ao usuário."""
    portas = list(serial.tools.list_ports.comports())
    candidatas = [p for p in portas if 'Arduino' in (p.description or '')
                  or 'CH340' in (p.description or '')
                  or 'USB' in (p.description or '')]
    if candidatas:
        return candidatas[0].device
    if portas:
        print("Portas disponíveis:")
        for i, p in enumerate(portas):
            print(f"  [{i}] {p.device} — {p.description}")
        idx = int(input("Escolha o número da porta: "))
        return portas[idx].device
    raise RuntimeError("Nenhuma porta serial encontrada.")


def main():
    # Nome do arquivo de saída com timestamp
    timestamp  = datetime.now().strftime("%Y%m%d_%H%M%S")
    nome_arq   = f"dados_malha_aberta_{timestamp}.csv"
    caminho    = Path(__file__).parent / nome_arq

    # Detecta e abre a porta serial
    porta = detectar_porta()
    print(f"[INFO] Conectando em {porta} @ {BAUD_RATE} baud…")

    with serial.Serial(porta, BAUD_RATE, timeout=TIMEOUT_S) as ser:
        # Aguarda o Arduino inicializar (reset ao abrir porta)
        time.sleep(2.5)
        ser.reset_input_buffer()

        # Lê mensagem de aguardo do Arduino
        linha_boas_vindas = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"[Arduino] {linha_boas_vindas}")

        # Envia comando de início
        ser.write(INICIO_CMD)
        print("[INFO] Sinal de início enviado. Coletando dados…")
        print(f"[INFO] Salvando em: {caminho}")

        linhas_salvas = 0
        cabecalho_ok  = False

        with open(caminho, 'w', newline='', encoding='utf-8') as arq:
            writer = None

            while True:
                try:
                    raw = ser.readline()
                except serial.SerialException as e:
                    print(f"[ERRO] Serial: {e}")
                    break

                if not raw:
                    continue  # timeout sem dado

                linha = raw.decode('utf-8', errors='ignore').strip()
                if not linha:
                    continue

                # Linhas de log do Arduino (começam com #)
                if linha.startswith(COMENTARIO):
                    print(f"[Arduino] {linha}")
                    if 'FIM DO ENSAIO' in linha:
                        break
                    continue

                # Cabeçalho CSV
                if not cabecalho_ok:
                    colunas = [c.strip() for c in linha.split(',')]
                    writer  = csv.writer(arq)
                    writer.writerow(colunas)
                    cabecalho_ok = True
                    print(f"[INFO] Colunas: {colunas}")
                    continue

                # Linhas de dados
                campos = linha.split(',')
                if writer and len(campos) == 4:
                    writer.writerow(campos)
                    linhas_salvas += 1
                    # Feedback a cada 50 amostras
                    if linhas_salvas % 50 == 0:
                        print(f"[INFO] {linhas_salvas} amostras salvas…  "
                              f"α={campos[2]} rad", end='\r')

    print(f"\n[OK] Ensaio encerrado. {linhas_salvas} amostras salvas em '{caminho}'.")
    return caminho


if __name__ == '__main__':
    try:
        arquivo_gerado = main()
        print(f"\nPróximo passo: execute  python estimar_tf.py \"{arquivo_gerado}\"")
    except KeyboardInterrupt:
        print("\n[AVISO] Interrompido pelo usuário.")
        sys.exit(0)
    except Exception as e:
        print(f"[ERRO] {e}")
        sys.exit(1)
