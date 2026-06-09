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
     ou com porta explícita:
       python coletar_dados.py --port COM3

DEPENDÊNCIAS:
  pip install pyserial

SAÍDA:
  dados_malha_aberta_YYYYMMDD_HHMMSS.csv
  (mesmo diretório do script)
============================================================
"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import serial
import serial.tools.list_ports

# ── Configurações ─────────────────────────────────────────────
BAUD_RATE = 9600
TIMEOUT_S = 2           # Timeout de leitura serial (s)
INICIO_CMD = b'S'       # Comando de início enviado ao Arduino
COMENTARIO = '#'        # Linhas de log (não são dados)


def listar_portas() -> List:
    """Retorna a lista de portas seriais disponíveis."""
    return list(serial.tools.list_ports.comports())


def detectar_porta(preferida: Optional[str] = None) -> str:
    """Retorna a porta COM para conectarse ao Arduino."""
    portas = listar_portas()
    if preferida:
        for p in portas:
            if p.device.lower() == preferida.lower():
                return p.device
        print(f"[AVISO] Porta {preferida} não encontrada entre as portas disponíveis.")

    candidatas = [p for p in portas if 'arduino' in (p.description or '').lower()
                  or 'ch340' in (p.description or '').lower()
                  or 'usb' in (p.description or '').lower()]
    if candidatas:
        return candidatas[0].device

    if portas:
        print("Portas disponíveis:")
        for i, p in enumerate(portas):
            print(f"  [{i}] {p.device} — {p.description}")
        while True:
            escolha = input("Escolha o número da porta ou digite o nome (ex: COM3): ").strip()
            if escolha.isdigit():
                idx = int(escolha)
                if 0 <= idx < len(portas):
                    return portas[idx].device
            elif escolha:
                return escolha
            print("Entrada inválida. Tente novamente.")

    raise RuntimeError("Nenhuma porta serial encontrada. Verifique a conexão do Arduino.")


def abrir_serial(porta: str, baud: int, timeout: float) -> serial.Serial:
    """Abre a porta serial com tratamento de erros comuns."""
    try:
        return serial.Serial(porta, baud, timeout=timeout)
    except serial.SerialException as e:
        raise RuntimeError(
            f"Não foi possível abrir {porta} em {baud} baud: {e}. "
            "Verifique se a porta está livre e se o Monitor Serial está fechado.")
    except PermissionError as e:
        raise RuntimeError(
            f"Permissão negada ao abrir {porta}: {e}. "
            "Execute o script como administrador ou libere a porta.")


def criar_nome_arquivo(prefixo: str = 'dados_malha_aberta') -> Path:
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    nome = f"{prefixo}_{timestamp}.csv"
    return Path(__file__).resolve().parent / nome


def processar_linha(linha: str, writer: Optional[csv.writer], cabecalho_ok: bool) -> Tuple[bool, int, Optional[csv.writer]]:
    """Processa uma linha serial e grava no CSV se for dado válido."""
    if linha.startswith(COMENTARIO):
        print(f"[Arduino] {linha}")
        return cabecalho_ok, 0, writer

    campos = [c.strip() for c in linha.split(',') if c.strip()]
    if not campos:
        return cabecalho_ok, 0, writer

    if not cabecalho_ok:
        writer.writerow(campos)
        print(f"[INFO] Colunas: {campos}")
        return True, 0, writer

    writer.writerow(campos)
    return cabecalho_ok, 1, writer


def main() -> Path:
    parser = argparse.ArgumentParser(description='Coleta dados do pêndulo de Furuta via Arduino Mega.')
    parser.add_argument('--port', help='Porta serial COM do Arduino (ex: COM3)')
    parser.add_argument('--baud', type=int, default=BAUD_RATE, help='Taxa de transmissão serial')
    parser.add_argument('--timeout', type=float, default=TIMEOUT_S, help='Timeout da leitura serial em segundos')
    parser.add_argument('--output', help='Caminho do arquivo CSV de saída')
    args = parser.parse_args()

    porta = detectar_porta(args.port)
    print(f"[INFO] Conectando em {porta} @ {args.baud} baud…")

    caminho = Path(args.output) if args.output else criar_nome_arquivo()
    caminho.parent.mkdir(parents=True, exist_ok=True)

    with abrir_serial(porta, args.baud, args.timeout) as ser:
        time.sleep(2.5)
        ser.reset_input_buffer()

        boas_vindas = ser.readline().decode('utf-8', errors='ignore').strip()
        if boas_vindas:
            print(f"[Arduino] {boas_vindas}")

        ser.write(INICIO_CMD)
        ser.flush()
        print("[INFO] Sinal de início enviado. Coletando dados…")
        print(f"[INFO] Salvando em: {caminho}")

        linhas_salvas = 0
        cabecalho_ok = False

        with open(caminho, 'w', newline='', encoding='utf-8') as arq:
            writer = csv.writer(arq)

            while True:
                try:
                    raw = ser.readline()
                except serial.SerialException as e:
                    print(f"[ERRO] Serial: {e}")
                    break

                if not raw:
                    continue

                linha = raw.decode('utf-8', errors='ignore').strip()
                if not linha:
                    continue

                if linha.startswith(COMENTARIO):
                    print(f"[Arduino] {linha}")
                    if 'FIM DO ENSAIO' in linha.upper():
                        break
                    continue

                if not cabecalho_ok:
                    colunas = [c.strip() for c in linha.split(',') if c.strip()]
                    if colunas:
                        writer.writerow(colunas)
                        cabecalho_ok = True
                        print(f"[INFO] Colunas: {colunas}")
                    continue

                dados = [c.strip() for c in linha.split(',')]
                if dados:
                    writer.writerow(dados)
                    linhas_salvas += 1
                    if linhas_salvas % 50 == 0:
                        valor = dados[2] if len(dados) > 2 else ''
                        print(f"[INFO] {linhas_salvas} amostras salvas… α={valor} rad", end='\r')

    print(f"\n[OK] Ensaio encerrado. {linhas_salvas} amostras salvas em '{caminho}'.")
    return caminho


if __name__ == '__main__':
    try:
        arquivo_gerado = main()
        print(f"\nPróximo passo: execute python estimar_tf.py \"{arquivo_gerado}\"")
    except KeyboardInterrupt:
        print("\n[AVISO] Interrompido pelo usuário.")
        sys.exit(0)
    except Exception as e:
        print(f"[ERRO] {e}")
        sys.exit(1)
