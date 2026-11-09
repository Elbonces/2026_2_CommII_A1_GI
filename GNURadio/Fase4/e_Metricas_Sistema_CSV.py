"""
e_Metricas_Sistema_CSV  -  metricas del enlace por punto del barrido de Eb/N0 (Guia 4A), grupo A1-GI.

Implementacion propia con las mismas columnas del CSV de la guia. Todas las entradas llegan A LA TASA DE SIMBOLO
(en el flujograma, las dos I/Q pasan por un Keep 1 in N con N = bits_por_simbolo; asi ninguna entrada
tiene tasa distinta y no se detiene el flujograma):

  0  I/Q limpia (antes de e_EbN0_Sweep, decimada 1 de cada N)
  1  I/Q con ruido (salida de e_EbN0_Sweep, decimada 1 de cada N)
  2  indice de simbolo Tx (0..15)   -> salida 1 de e_Alinea_Ref
  3  indice de simbolo Rx (0..15)   -> salida 0 de e_Alinea_Ref
  4  Eb/N0 actual (salida naranja de e_EbN0_Sweep, decimada 1 de cada N)

BER: cada indice de simbolo son exactamente sus bps bits (MSB primero), por lo que los bits erroneos
son popcount(tx XOR rx): resultado identico a comparar los bits desempaquetados. Por eso no hay entradas de bits
(a tasa de bit distinta de la de simbolo detendrian el flujograma); la guia pide bits Tx/Rx, que aqui entran
ya agrupados como indice de simbolo.

Cada fila se escribe al completar una ventana (bits_guardia + bits_per_record bits, convertida a simbolos),
la misma ventana que usan e_BER_Curve / e_SER_Curve, por lo que una fila = un unico punto Eb/N0.

Columnas: modulation_type, bits_per_symbol_m, samp_rate_hz, symbol_rate_rs_baud, bandwidth_bw_hz,
amplitude_a_v, ebn0_db (nivel del barrido al que pertenece la ventana), noise_variance_sigma2 (potencia de y-x, I/Q, medida en ese nivel),
measured_snr_db (10log10 P_limpia / sigma2), measured_ber, measured_ser,
iq_dispersion_rms (sqrt(mean|y-x|^2 / mean|x|^2): dispersion rms normalizada de la I/Q con ruido).
"""
import csv
import numpy as np
from pathlib import Path
from gnuradio import gr

POP = np.array([bin(i).count('1') for i in range(16)], dtype=np.int64)
COLS = ['modulation_type', 'bits_per_symbol_m', 'samp_rate_hz', 'symbol_rate_rs_baud', 'bandwidth_bw_hz',
        'amplitude_a_v', 'ebn0_db', 'noise_variance_sigma2', 'measured_snr_db', 'measured_ber',
        'measured_ser', 'iq_dispersion_rms']


class blk(gr.sync_block):
    def __init__(self, modulation_type='16QAM', bits_per_symbol_m=4, samp_rate_hz=2.5e6,
                 symbol_rate_rs_baud=625e3, bandwidth_bw_hz=844e3, amplitude_a_v=1.0,
                 bits_per_record=100000, bits_guardia=20000, delay_bits=0, delay_symbols=0,
                 EbN0_min=0.0, EbN0_max=14.0, EbN0_step=1.0,
                 csv_filename='metricas_sistema_GrupoA1-GI.csv', overwrite_on_start=True):
        gr.sync_block.__init__(self, name='e_Metricas_Sistema_CSV',
                               in_sig=[np.complex64, np.complex64, np.uint8, np.uint8, np.float32],
                               out_sig=None)
        self.mod = str(modulation_type)
        self.m = max(1, int(bits_per_symbol_m))
        self.fs = float(samp_rate_hz); self.rs = float(symbol_rate_rs_baud)
        self.bw = float(bandwidth_bw_hz); self.amp = float(amplitude_a_v)
        self.guard = max(0, int(bits_guardia)) // self.m
        self.rec = max(1, int(bits_per_record)) // self.m
        self.seg = self.guard + self.rec
        self.ret = max(0, int(delay_symbols))
        self.n = int(round((float(EbN0_max) - float(EbN0_min)) / float(EbN0_step))) + 1
        self.eb0, self.step = float(EbN0_min), float(EbN0_step)
        base = Path(__file__).resolve().parent if '__file__' in globals() else Path.cwd()
        self.ruta = base / str(csv_filename)
        if overwrite_on_start or not self.ruta.exists():
            with open(self.ruta, 'w', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow(COLS)
        self.k = 0                      # simbolos procesados (entrada 3)
        self.bx = np.zeros(self.n); self.bd = np.zeros(self.n); self.bn = np.zeros(self.n, dtype=np.int64)
        self._desde = 0
        self._reset()
        self._tx_hist = np.zeros(self.ret, dtype=np.uint8)
        print('CSV de metricas:', self.ruta, flush=True)

    def _reset(self):
        self.errb = 0; self.errs = 0; self.nsym = 0

    def _flush(self):
        p = (self.k // self.seg - 1) % self.n            # punto del barrido al que pertenece la ventana
        n = max(self.nsym, 1)
        c = max(self.bn[p], 1)
        px = self.bx[p] / c; s2 = self.bd[p] / c
        self.bx[p] = self.bd[p] = 0.0; self.bn[p] = 0
        snr = 10 * np.log10(px / s2) if s2 > 0 and px > 0 else float('nan')
        disp = float(np.sqrt(s2 / px)) if px > 0 else float('nan')
        fila = [self.mod, self.m, self.fs, self.rs, self.bw, self.amp, '%.3f' % (self.eb0 + self.step * p), '%.6e' % s2, '%.3f' % snr,
                '%.6e' % (self.errb / (n * self.m)), '%.6e' % (self.errs / n), '%.6f' % disp]
        with open(self.ruta, 'a', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(fila)
        self._reset()

    def _estadisticas_iq(self, x, y, eb):
        """Acumula potencia limpia y de ruido (y-x) en el contenedor de su nivel de Eb/N0, ignorando los
        primeros bits_guardia/m simbolos tras cada cambio de nivel (las I/Q no llevan el retardo del receptor)."""
        N = len(x)
        cam = np.flatnonzero(np.diff(eb) != 0.0) + 1
        ult = np.zeros(N, dtype=np.int64); ult[cam] = cam
        ult = np.maximum.accumulate(ult)
        desde = np.where(ult > 0, np.arange(N) - ult, self._desde + np.arange(N))
        self._desde = int(desde[-1]) + 1
        ok = desde >= self.guard
        b = np.rint((eb - self.eb0) / self.step).astype(np.int64)
        ok &= (b >= 0) & (b < self.n) & (np.abs(eb - (self.eb0 + b * self.step)) < 1e-3)
        if not ok.any():
            return
        xx = x[ok].astype(np.complex128); d = y[ok].astype(np.complex128) - xx; bb = b[ok]
        self.bx += np.bincount(bb, weights=np.abs(xx) ** 2, minlength=self.n)
        self.bd += np.bincount(bb, weights=np.abs(d) ** 2, minlength=self.n)
        self.bn += np.bincount(bb, minlength=self.n)

    def work(self, input_items, output_items):
        x, y, tx, rx, eb = input_items
        N = len(x)
        tx = tx & 15; rx = rx & 15
        if self.ret:     # retardo opcional de la referencia Tx
            full = np.concatenate([self._tx_hist, tx]); tx = full[:N]; self._tx_hist = full[N:]
        self._estadisticas_iq(x, y, eb)
        i = 0
        while i < N:
            pos = self.k % self.seg
            tomar = min(N - i, self.seg - pos)
            if pos < self.guard:
                tomar = min(tomar, self.guard - pos)
            else:
                sl = slice(i, i + tomar)
                diff = tx[sl] ^ rx[sl]
                self.errb += int(POP[diff].sum()); self.errs += int(np.count_nonzero(diff)); self.nsym += tomar
            self.k += tomar; i += tomar
            if self.k % self.seg == 0:
                self._flush()
        return N
