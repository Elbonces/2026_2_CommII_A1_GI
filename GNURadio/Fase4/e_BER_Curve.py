"""
e_BER_Curve (version rapida, mismo comportamiento que el archivo suministrado)
Acumulador de curva BER para los sistemas de los estudiantes.

Entrada 0 : bits transmitidos de referencia (byte 0/1).
Entrada 1 : bits recuperados por el receptor (byte 0/1).

Salida 0  : vector BER lineal.
Salida 1  : vector log10(BER) para QT GUI Vector Sink.

El bloque usa exactamente el mismo barrido que e_EbN0_Sweep. Para cada punto:
- ignora bits_guardia al comienzo del segmento;
- compara bits_por_punto;
- cuando el barrido vuelve a un punto, sigue acumulando estadísticas.

delay_bits retrasa internamente la referencia Tx para compensar el retardo
del receptor. Se ajusta primero en una condición limpia hasta lograr BER≈0.

Cuando todavía no se observa ningún error, la salida logarítmica utiliza
3/N como cota superior aproximada para poder mostrar el punto sin afirmar
que BER=0.
"""

import numpy as np
from gnuradio import gr


class blk(gr.decim_block):
    def __init__(self,
                 EbN0_min=0.0,
                 EbN0_max=14.0,
                 EbN0_step=1.0,
                 bits_por_punto=100000,
                 bits_guardia=1000,
                 delay_bits=0,
                 update_every=1000):
        self.EbN0_min = float(EbN0_min)
        self.EbN0_max = float(EbN0_max)
        self.EbN0_step = float(EbN0_step)
        self.bits_por_punto = max(1, int(bits_por_punto))
        self.bits_guardia = max(0, int(bits_guardia))
        self.delay_bits = max(0, int(delay_bits))
        self.update_every = max(1, int(update_every))

        if self.EbN0_step <= 0:
            raise ValueError("EbN0_step debe ser > 0")
        if self.EbN0_max < self.EbN0_min:
            raise ValueError("EbN0_max debe ser >= EbN0_min")

        npts = int(round((self.EbN0_max-self.EbN0_min)/self.EbN0_step)) + 1
        self.eb_axis = self.EbN0_min + self.EbN0_step*np.arange(npts, dtype=np.float64)
        self.N = len(self.eb_axis)
        self.segment_bits = self.bits_guardia + self.bits_por_punto

        gr.decim_block.__init__(
            self,
            name='e_BER_Curve',
            in_sig=[np.int8, np.int8],
            out_sig=[(np.float32, self.N), (np.float32, self.N)],
            decim=self.update_every
        )

        self.carry = np.zeros(self.delay_bits, dtype=np.uint8)
        self.raw_items = 0
        self.valid_bit_index = 0

        self.count = np.zeros(self.N, dtype=np.int64)
        self.errors = np.zeros(self.N, dtype=np.int64)

    def _process(self, tx, rx):
        # Version vectorizada de la original (mismo resultado, ~100x mas rapida):
        # el bucle bit a bit en Python no alcanza para Rb = 2,5 Mb/s en tiempo real.
        tb = (np.asarray(tx, dtype=np.int64) & 1).astype(np.uint8)
        rb = (np.asarray(rx, dtype=np.int64) & 1).astype(np.uint8)
        n = len(tb)
        if self.delay_bits > 0:
            lleno = np.concatenate([self.carry, tb])
            ta = lleno[:n]
            self.carry = lleno[n:]
            salto = min(n, max(0, self.delay_bits - self.raw_items))
            ta, rb = ta[salto:], rb[salto:]
        else:
            ta = tb
        self.raw_items += n
        m = len(ta)
        if m == 0:
            return
        idx = self.valid_bit_index + np.arange(m, dtype=np.int64)
        punto = (idx // self.segment_bits) % self.N
        pos = idx % self.segment_bits
        ok = pos >= self.bits_guardia
        self.count += np.bincount(punto[ok], minlength=self.N)
        self.errors += np.bincount(punto[ok & (ta != rb)], minlength=self.N)
        self.valid_bit_index += m

    def _vectors(self):
        ber = np.zeros(self.N, dtype=np.float32)
        logber = np.zeros(self.N, dtype=np.float32)

        for i in range(self.N):
            n = int(self.count[i])
            e = int(self.errors[i])

            if n <= 0:
                ber[i] = 0.0
                logber[i] = 0.0
            elif e > 0:
                b = e/float(n)
                ber[i] = b
                logber[i] = np.log10(max(b, 1e-12))
            else:
                # BER observado = 0; para la gráfica usamos una cota superior.
                ber[i] = 0.0
                upper = min(1.0, 3.0/float(n))
                logber[i] = np.log10(max(upper, 1e-12))

        return ber, logber

    def work(self, input_items, output_items):
        tx = input_items[0]
        rx = input_items[1]
        out_ber = output_items[0]
        out_log = output_items[1]

        nout = len(out_ber)
        D = self.update_every

        for k in range(nout):
            a = k*D
            b = min((k+1)*D, len(tx))
            if a < b:
                self._process(tx[a:b], rx[a:b])

            ber, logber = self._vectors()
            out_ber[k, :] = ber
            out_log[k, :] = logber

        return nout
