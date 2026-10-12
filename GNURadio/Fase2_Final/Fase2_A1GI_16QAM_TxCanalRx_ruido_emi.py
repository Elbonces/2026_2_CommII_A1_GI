"""
Ruido impulsivo EMI (modelo Bernoulli-Gaussiano por rafagas).

Representa las rafagas de interferencia que producen los variadores de
frecuencia (VFD) y equipos de soldadura de la celda: cada conmutacion genera
una rafaga de ruido gaussiano complejo de duracion 'dur'. Las rafagas llegan
como un proceso de Poisson de tasa 'rate' (rafagas/s). Fuera de las rafagas
la salida es cero (el ruido de fondo lo aporta el bloque AWGN).

Parametros
  samp_rate : tasa de muestreo [muestras/s]
  rate      : rafagas por segundo (~2*f_PWM del variador)
  dur       : duracion de cada rafaga [s]
  amp       : amplitud RMS compleja del ruido dentro de la rafaga
  enable    : 1 = activo, 0 = apagado
"""
import numpy as np
from gnuradio import gr


class blk(gr.sync_block):
    def __init__(self, samp_rate=24e6, rate=16e3, dur=1e-6, amp=1.0, enable=0):
        gr.sync_block.__init__(self, name='Ruido impulsivo EMI',
                               in_sig=None, out_sig=[np.complex64])
        self.samp_rate = samp_rate
        self.rate = rate
        self.dur = dur
        self.amp = amp
        self.enable = enable
        self._rng = np.random.default_rng(12345)
        self._resto = 0  # muestras pendientes de una rafaga que viene del buffer anterior

    def work(self, input_items, output_items):
        out = output_items[0]
        n = len(out)
        out[:] = 0
        if (not self.enable) or self.amp <= 0:
            self._resto = 0
            return n
        L = max(1, int(round(self.dur * self.samp_rate)))
        p = min(1.0, self.rate / self.samp_rate)
        mask = np.zeros(n, dtype=bool)
        mask[:min(self._resto, n)] = True
        resto = max(self._resto - n, 0)
        for s in np.flatnonzero(self._rng.random(n) < p):
            mask[s:s + L] = True
            resto = max(resto, s + L - n)
        self._resto = resto
        m = int(mask.sum())
        if m:
            ruido = (self._rng.standard_normal(m) + 1j * self._rng.standard_normal(m)) * (self.amp / np.sqrt(2))
            out[mask] = ruido.astype(np.complex64)
        return n
