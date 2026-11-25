# Introdução a Sistemas de Controle e Pêndulo de Furuta

Laboratórios 2 a 13 (MATLAB/Simulink, tanque atmosférico, PID, PID 2-DOF, feedforward e cascata), identificação em malha aberta e trabalho final do pêndulo de Furuta, com firmware Arduino, scripts de coleta serial e relatórios.

## Contexto
Disciplina de Introdução aos Sistemas de Controle (UFMT, 2026/1) e trabalho final.

## Estrutura
- `arduino-sketches/` (2 arquivos)
- `codigos-soltos/` (17 arquivos; inclui `estimar_tf_furuta.py`, estimação da FT com referência para baixo)
- `coleta-dados-furuta/` (25 arquivos)
- `furuta-pastas-do-downloads/` (5 arquivos)
- `furuta-sistemadecontrole/` (12 arquivos)
- `labs-soltos/` (21 arquivos)
- `testes-furuta-pid2dof/`: experimento de PID de 2 graus de liberdade no Furuta (modelos no MATLAB, calibração do modelo não linear em Python e firmware Arduino), de 14/07/2026
- `material-de-downloads/` (62 arquivos)
- `matlab-documentos/` (17 arquivos)
- `matlab-sistemas-de-controle/` (86 arquivos)
- `matlab-trabalho-final-modelos/` (52 arquivos)
- `trabalho-final-furuta/` (15 arquivos)

## Conteúdo
- 313 arquivos (289.3 MB no total).
- Tipos mais comuns: .mat ×58, .png ×57, .m ×45, .slx ×30, .pdf ×20, .py ×15, .slxc ×13, .csv ×13.
- Datas dos arquivos: 2026-06-08 a 2026-11-24.

## Como executar
- Python: `pip install -r requirements.txt` (na pasta que contém o arquivo).
- MATLAB/Simulink: abrir os `.m` e `.slx` (os labs usam Simulink).
- Arduino/ESP32: abrir os `.ino` na Arduino IDE ou PlatformIO; `wokwi-*` roda no simulador Wokwi.

## Estado e observações
**Versão final do trabalho do Furuta (conferida por hash):** `trabalho-final-furuta\V3\trabalho_final_furuta_v3.m` (há cópias idênticas em `matlab-sistemas-de-controle\TrabalhoFinal\V3` e `codigos-soltos`) com `build_modelos_furuta_v4.m` (o mais recente, em `matlab-sistemas-de-controle\TrabalhoFinal\V3`). Linha do tempo: v1 em 07/07 (`build_modelo_furuta` + `trabalho_final_furuta`), v2 em 08/07, variantes de 14/07 em `material-de-downloads`, v3 de 15 a 17/07. `codigos-soltos\trabalho_final_furuta_v3_variante.m` é uma variante do mesmo dia, com hash diferente. Há 17 grupos de arquivos idênticos entre as subpastas (ver `log_duvidas_RESOLUCAO.md`). Os nomes de `codigos-soltos` foram corrigidos em 25/11 (sem data e sem hífen) para o MATLAB conseguir chamá-los. `furuta-sistemadecontrole\.venv` parou ao mudar de pasta: recrie com `pip install -r requirements.txt`.

## Origem dos arquivos
Reorganização feita entre 24 e 25/11/2026. De onde veio cada parte:
- `arduino-sketches` ← `Área de Trabalho\Projetos-faculdade-codigo\Arduino-sketches`
- `coleta-dados-furuta` ← `Área de Trabalho\Projetos-faculdade-codigo\Faculdade-Robotica\Controle`
- `furuta-pastas-do-downloads\furuta_malha_aberta` ← `Área de Trabalho\Projetos-faculdade-codigo\Faculdade-Robotica\Pastas-do-Downloads\furuta_malha_aberta`
- `furuta-pastas-do-downloads\furuta_pid2dof` ← `Área de Trabalho\Projetos-faculdade-codigo\Faculdade-Robotica\Pastas-do-Downloads\furuta_pid2dof`
- `furuta-pastas-do-downloads\PenduloInvertidoRotacional_Controle` ← `Área de Trabalho\Projetos-faculdade-codigo\Faculdade-Robotica\Pastas-do-Downloads\PenduloInvertidoRotacional_Controle`
- `furuta-pastas-do-downloads\POSICOESEVELOCIDADESFURUTA` ← `Área de Trabalho\Projetos-faculdade-codigo\Faculdade-Robotica\Pastas-do-Downloads\POSICOESEVELOCIDADESFURUTA`
- `codigos-soltos\analise_malha_aberta.py` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-06-08_analise-malha-aberta.py`
- `codigos-soltos\captura_serial_variante.py` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-06-08_captura-serial-1.py`
- `codigos-soltos\captura_serial.py` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-06-08_captura-serial.py`
- `codigos-soltos\coletar_dados.py` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-06-08_coletar-dados.py`
- `codigos-soltos\estimar_tf.py` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-06-08_estimar-tf.py`
- `codigos-soltos\furuta_malha_aberta.ino` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-06-08_furuta-malha-aberta.ino`
- `codigos-soltos\aula_08_lab.slx` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-06-12_aula-08.slx`
- `codigos-soltos\aula08_lab.m` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-06-12_aula08-lab.m`
- `codigos-soltos\aula08.m` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-06-12_aula08.m`
- `codigos-soltos\aula_10.slx` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-07-03_aula-10.slx`
- `codigos-soltos\build_modelo_furuta.m` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-07-07_build-modelo-furuta.m`
- `codigos-soltos\modelo_furuta.slx` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-07-07_modelo-furuta.slx`
- `codigos-soltos\trabalho_final_furuta.m` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-07-07_trabalho-final-furuta.m`
- `codigos-soltos\build_modelos_furuta_v2.m` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-07-15_build-modelos-furuta.m`
- `codigos-soltos\trabalho_final_furuta_v3_variante.m` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-07-15_trabalho-final-furuta-1.m`
- `codigos-soltos\trabalho_final_furuta_v3.m` ← `Área de Trabalho\Projetos-faculdade-codigo\Codigos-soltos\2026-07-15_trabalho-final-furuta-v3.m`
- `matlab-sistemas-de-controle` ← `Área de Trabalho\MatLabTrabalhos\Sistemas de Controle`
- `matlab-trabalho1` ← `Área de Trabalho\MatLabTrabalhos\Trabalho1`
- `labs-soltos\aula_07.slx` ← `Área de Trabalho\MatLabTrabalhos\aula_07.slx`
- `labs-soltos\aula_07.slx.r2024a` ← `Área de Trabalho\MatLabTrabalhos\aula_07.slx.r2024a`
- `labs-soltos\aula_07.slxc` ← `Área de Trabalho\MatLabTrabalhos\aula_07.slxc`
- `labs-soltos\aula_09.slx` ← `Área de Trabalho\MatLabTrabalhos\aula_09.slx`
- `labs-soltos\aula_09.slxc` ← `Área de Trabalho\MatLabTrabalhos\aula_09.slxc`
- `labs-soltos\aula_11.slx` ← `Área de Trabalho\MatLabTrabalhos\aula_11.slx`
- `labs-soltos\ContainerScopePrintToFigure.png` ← `Área de Trabalho\MatLabTrabalhos\ContainerScopePrintToFigure.png`
- `labs-soltos\Figure_1_hyst0.005.png` ← `Área de Trabalho\MatLabTrabalhos\Figure_1_hyst0.005.png`
- `labs-soltos\Figure_1_hyst0.02.png` ← `Área de Trabalho\MatLabTrabalhos\Figure_1_hyst0.02.png`
- `labs-soltos\Figure_1_hyst0.08.png` ← `Área de Trabalho\MatLabTrabalhos\Figure_1_hyst0.08.png`
- `labs-soltos\lab09.m` ← `Área de Trabalho\MatLabTrabalhos\lab09.m`
- `labs-soltos\Lab11.m` ← `Área de Trabalho\MatLabTrabalhos\Lab11.m`
- `labs-soltos\Lab11_fig1.png` ← `Área de Trabalho\MatLabTrabalhos\Lab11_fig1.png`
- `labs-soltos\Lab11_fig2.png` ← `Área de Trabalho\MatLabTrabalhos\Lab11_fig2.png`
- `labs-soltos\Lab11_fig3.png` ← `Área de Trabalho\MatLabTrabalhos\Lab11_fig3.png`
- `labs-soltos\Lab11_pronto.m` ← `Área de Trabalho\MatLabTrabalhos\Lab11_pronto.m`
- `labs-soltos\Lab9Fig1.png` ← `Área de Trabalho\MatLabTrabalhos\Lab9Fig1.png`
- `labs-soltos\Lab9Fig2.png` ← `Área de Trabalho\MatLabTrabalhos\Lab9Fig2.png`
- `labs-soltos\Lab9Fig3.png` ← `Área de Trabalho\MatLabTrabalhos\Lab9Fig3.png`
- `labs-soltos\Lab9Fig4.png` ← `Área de Trabalho\MatLabTrabalhos\Lab9Fig4.png`
- `labs-soltos\Lab_7.m` ← `Área de Trabalho\MatLabTrabalhos\Lab_7.m`
- `furuta-sistemadecontrole` ← `Área de Trabalho\SistemaDeControle`
- `trabalho-final-furuta` ← `Área de Trabalho\TrabalhoFinal`
- `material-de-downloads\aula-09-lab09` ← `Downloads\Aula 09`
- `material-de-downloads\nova-pasta-furuta-arquivos-soltos` ← `Downloads\Nova pasta`
- `material-de-downloads\testes-furuta-pid2dof` ← `Downloads\testandosamerda`
- `material-de-downloads\2026-06-19_03-a20-lab09-roteiro.pdf` ← `Downloads\03_A20_Lab09_Roteiro.pdf`
- `material-de-downloads\04_A24_Lab11_PID_Discreto.m` ← `Downloads\04_A24_Lab11_PID_Discreto.m`
- `material-de-downloads\2026-06-26_07-a22-lab10-roteiro.pdf` ← `Downloads\07_A22_Lab10_Roteiro.pdf`
- `material-de-downloads\Aula 09.zip` ← `Downloads\Aula 09.zip`
- `material-de-downloads\codigo_din_matlab_function_linha_a_linha.html` ← `Downloads\codigo_din_matlab_function_linha_a_linha.html`
- `material-de-downloads\2026-07-16_conteudo-relatorio-furuta.md` ← `Downloads\conteudo_relatorio_furuta.md`
- `material-de-downloads\2026-07-16_diagrama-classico-modelo1-linear.png` ← `Downloads\diagrama_classico_modelo1_linear.png`
- `material-de-downloads\2026-07-16_diagrama-classico-modelo2-deadzone.png` ← `Downloads\diagrama_classico_modelo2_deadzone.png`
- `material-de-downloads\2026-07-16_diagrama-classico-modelo3-naolinear.png` ← `Downloads\diagrama_classico_modelo3_naolinear.png`
- `material-de-downloads\estimar_tf_furuta.py` ← `Downloads\estimar_tf_furuta.py`
- `material-de-downloads\explorador_ino_furuta_parte_a_parte.html` ← `Downloads\explorador_ino_furuta_parte_a_parte.html`
- `material-de-downloads\isr_controle_linha_a_linha.html` ← `Downloads\isr_controle_linha_a_linha.html`
- `material-de-downloads\isr_encoder_linha_a_linha.html` ← `Downloads\isr_encoder_linha_a_linha.html`
- `material-de-downloads\2026-06-20_lab09-relatorio-erick.docx` ← `Downloads\Lab09_Relatorio_Erick.docx`
- `material-de-downloads\2026-07-03_lab11-respostas-poe.md` ← `Downloads\Lab11_Respostas_POE.md`
- `material-de-downloads\2026-07-03_lab11-roteiro.pdf` ← `Downloads\Lab11_Roteiro.pdf`
- `material-de-downloads\2026-07-17_lab12relatorio.pdf` ← `Downloads\Lab12Relatório.pdf`
- `material-de-downloads\2026-07-15_lab12-relatorio.pdf` ← `Downloads\Lab12_Relatório.pdf`
- `material-de-downloads\2026-07-15_lab12-respostas.md` ← `Downloads\Lab12_Respostas.md`
- `material-de-downloads\2026-07-15_lab12-roteiro.pdf` ← `Downloads\Lab12_Roteiro.pdf`
- `material-de-downloads\2026-07-17_lab13-folha-poe.md` ← `Downloads\Lab13_folha_POE.md`
- `material-de-downloads\2026-07-17_lab13-roteiro-poe.pdf` ← `Downloads\Lab13_roteiro_POE.pdf`
- `material-de-downloads\2026-07-14_laboratorios-de-sistema-de-controle.md` ← `Downloads\LABORATÓRIOS DE SISTEMA DE CONTROLE.md`
- `material-de-downloads\2026-07-14_mudancas-build-modelos-furuta.md` ← `Downloads\MUDANCAS_build_modelos_furuta.md`
- `material-de-downloads\2026-07-16_pid-1dof-vs-2dof-furuta.png` ← `Downloads\pid_1dof_vs_2dof_furuta.png`
- `material-de-downloads\pid_discreto_linha_a_linha.html` ← `Downloads\pid_discreto_linha_a_linha.html`
- `material-de-downloads\planta_nl_blocos_internos.html` ← `Downloads\planta_nl_blocos_internos.html`
- `material-de-downloads\RelatorioFinalPorFavor` ← `Downloads\RelatorioFinalPorFavor`
- `material-de-downloads\2026-06-20_relatoriolab9.pdf` ← `Downloads\RelatorioLab9.pdf`
- `material-de-downloads\relatorio_entrega2_fonte (1).zip` ← `Downloads\relatorio_entrega2_fonte (1).zip`
- `material-de-downloads\relatorio_entrega2_fonte.zip` ← `Downloads\relatorio_entrega2_fonte.zip`
- `material-de-downloads\2026-06-16_relatorio-entrega2-furuta.pdf` ← `Downloads\relatorio_entrega2_furuta (1).pdf`
- `material-de-downloads\2026-06-16_relatorio-entrega2-furuta-2.pdf` ← `Downloads\relatorio_entrega2_furuta.pdf`
- `material-de-downloads\2026-07-17_relatorio-firmware.pdf` ← `Downloads\relatorio_firmware.pdf`
- `material-de-downloads\2026-06-26_relatorio-lab10-pid2dof.docx` ← `Downloads\Relatorio_Lab10_PID2DOF.docx`
- `material-de-downloads\2026-07-17_relatorio-simulink.pdf` ← `Downloads\relatorio_simulink.pdf`
- `material-de-downloads\2026-06-16_relatoriosistemasdecontrole.pdf` ← `Downloads\RelatórioSistemasDeControle.pdf`
- `material-de-downloads\2026-07-03_relattorio-lab11.pdf` ← `Downloads\Relattorio_Lab11.pdf`
- `material-de-downloads\2026-07-17_relatoriolab13.pdf` ← `Downloads\RelatórioLab13.pdf`
- `material-de-downloads\2026-06-26_relatorio-lab10.pdf` ← `Downloads\Relatório_Lab10.pdf`
- `material-de-downloads\simulador_arguicao_furuta_pid.html` ← `Downloads\simulador_arguicao_furuta_pid.html`
- `material-de-downloads\simulink_dentro_pid_2dof.html` ← `Downloads\simulink_dentro_pid_2dof.html`
- `material-de-downloads\simulink_nivel_topo_bloco_a_bloco.html` ← `Downloads\simulink_nivel_topo_bloco_a_bloco.html`
- `material-de-downloads\testandosamerda.zip` ← `Downloads\testandosamerda.zip`
- `material-de-downloads\trabalho_final_furuta_v2.m` ← `Downloads\trabalho_final_furuta_v2.m`
- `material-de-downloads\2026-07-14_ttc-pendulo-de-furuta-kamylla-versao-final.pdf` ← `Downloads\TTC - PÊNDULO DE FURUTA - KAMYLLA Versão Final (2).pdf`
- `matlab-documentos` ← `Documentos\MATLAB\SistemaDeControle`
- `codigos-soltos\analise_malha_aberta.py` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-06-08_analise-malha-aberta.py`
- `codigos-soltos\captura_serial.py` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-06-08_captura-serial.py`
- `codigos-soltos\captura_serial_variante.py` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-06-08_captura-serial-1.py`
- `codigos-soltos\coletar_dados.py` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-06-08_coletar-dados.py`
- `codigos-soltos\estimar_tf.py` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-06-08_estimar-tf.py`
- `codigos-soltos\furuta_malha_aberta.ino` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-06-08_furuta-malha-aberta.ino`
- `codigos-soltos\aula08.m` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-06-12_aula08.m`
- `codigos-soltos\aula_08_lab.slx` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-06-12_aula-08.slx`
- `codigos-soltos\aula08_lab.m` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-06-12_aula08-lab.m`
- `codigos-soltos\aula_10.slx` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-07-03_aula-10.slx`
- `codigos-soltos\build_modelo_furuta.m` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-07-07_build-modelo-furuta.m`
- `codigos-soltos\modelo_furuta.slx` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-07-07_modelo-furuta.slx`
- `codigos-soltos\trabalho_final_furuta.m` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-07-07_trabalho-final-furuta.m`
- `codigos-soltos\build_modelos_furuta_v2.m` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-07-15_build-modelos-furuta.m`
- `codigos-soltos\trabalho_final_furuta_v3.m` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-07-15_trabalho-final-furuta-v3.m`
- `codigos-soltos\trabalho_final_furuta_v3_variante.m` ← `C:\Users\erick\Projetos\sistemas-de-controle\codigos-soltos\2026-07-15_trabalho-final-furuta-1.m`
- `coleta-dados-furuta\furuta_identificacao_degraus.ino` ← `C:\Users\erick\Projetos\sistemas-de-controle\coleta-dados-furuta\67.ino`
- `coleta-dados-furuta\monitor_tempo_real_furuta_v2.py` ← `C:\Users\erick\Projetos\sistemas-de-controle\coleta-dados-furuta\monitor2.py`
- `matlab-trabalho-final-modelos` ← `C:\Users\erick\Projetos\sistemas-de-controle\matlab-trabalho1\trabalhoFinal`
- `material-de-downloads\revisao-requisitos-trabalho-final-furuta.txt` ← `Downloads\message.txt`

_Detalhes completos em `log_movimentos_fase1.csv` a `log_movimentos_fase5.csv` (pasta `testesClaudeCode`)._