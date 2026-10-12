# Fase 2 – Enlace 16-QAM Tx–Canal–Rx (grupo A1-GI)

- `Fase2_A1GI_16QAM_TxCanalRx.grc`: flujograma final (GNU Radio 3.10). Ábralo y ejecútelo (F6).
  Controles: Es/N0 LOS, atenuación por sombra, potencia de la EMI, retardo de muestreo d,
  retardo de la referencia de BER, roll-off, y casillas para AWGN, EMI y AGC.
- La variable `modo_captura` = 1 activa el guion automático (bloque Python Snippet), que recorre
  todos los escenarios y guarda las capturas y `capturas/resultados.json` junto al .py generado.
  Con 0 (valor por defecto) el flujograma funciona de forma interactiva.
- `capturas/`: capturas Qt y resultados numéricos usados en el informe.
- `flujograma.pdf`: flujograma exportado desde GRC (vectorial).
- `figuras_informe.py`: genera la constelación Gray, la PSD por beta y la curva BER del informe.
