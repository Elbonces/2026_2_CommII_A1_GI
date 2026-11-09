"""
e_SER_Curve - Acumulador de curva SER (tasa de error de SIMBOLO) 16-QAM.
Adaptacion de e_BER_Curve: compara simbolos (0..15) en lugar de bits.

Entrada 0 : simbolo Tx de referencia (0..15), salida 1 de e_Alinea_Ref.
Entrada 1 : simbolo decidido por el receptor (0..15), salida 0 de e_Alinea_Ref.

Salida 0  : vector SER lineal.
Salida 1  : vector log10(SER) para QT GUI Vector Sink.

El bloque usa exactamente el mismo barrido que e_EbN0_Sweep. Para cada punto:
- ignora simbolos_guardia al comienzo del segmento;
- compara simbolos_por_punto;
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
                 simbolos_por_punto=25000,
                 simbolos_guardia=5000,
                 delay_simbolos=0,
                 update_every=1000):
        self.EbN0_min = float(EbN0_min)
        self.EbN0_max = float(EbN0_max)
        self.EbN0_step = float(EbN0_step)
        self.simbolos_por_punto = max(1, int(simbolos_por_punto))
        self.simbolos_guardia = max(0, int(simbolos_guardia))
        self.delay_simbolos = max(0, int(delay_simbolos))
        self.update_every = max(1, int(update_every))

        if self.EbN0_step <= 0:
            raise ValueError("EbN0_step debe ser > 0")
        if self.EbN0_max < self.EbN0_min:
            raise ValueError("EbN0_max debe ser >= EbN0_min")

        npts = int(round((self.EbN0_max-self.EbN0_min)/self.EbN0_step)) + 1
        self.eb_axis = self.EbN0_min + self.EbN0_step*np.arange(npts, dtype=np.float64)
        self.N = len(self.eb_axis)
        self.segment_bits = self.simbolos_guardia + self.simbolos_por_punto

        gr.decim_block.__init__(
            self,
            name='e_SER_Curve',
            in_sig=[np.uint8, np.uint8],
            out_sig=[(np.float32, self.N), (np.float32, self.N)],
            decim=self.update_every
        )

        self.carry = np.zeros(self.delay_simbolos, dtype=np.uint8)
        self.raw_items = 0
        self.valid_bit_index = 0

        self.count = np.zeros(self.N, dtype=np.int64)
        self.errors = np.zeros(self.N, dtype=np.int64)

    def _process(self, tx, rx):
        # Un simbolo es erroneo si el indice decidido difiere del transmitido.
        tb = (np.asarray(tx, dtype=np.int64) & 15).astype(np.uint8)
        rb = (np.asarray(rx, dtype=np.int64) & 15).astype(np.uint8)
        n = len(tb)
        if self.delay_simbolos > 0:
            lleno = np.concatenate([self.carry, tb])
            ta = lleno[:n]
            self.carry = lleno[n:]
            salto = min(n, max(0, self.delay_simbolos - self.raw_items))
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
        ok = pos >= self.simbolos_guardia
        self.count += np.bincount(punto[ok], minlength=self.N)
        self.errors += np.bincount(punto[ok & (ta != rb)], minlength=self.N)
        self.valid_bit_index += m

    def _vectors(self):
        ser = np.zeros(self.N, dtype=np.float32)
        logser = np.zeros(self.N, dtype=np.float32)

        for i in range(self.N):
            n = int(self.count[i])
            e = int(self.errors[i])

            if n <= 0:
                ser[i] = 0.0
                logser[i] = 0.0
            elif e > 0:
                b = e/float(n)
                ser[i] = b
                logser[i] = np.log10(max(b, 1e-12))
            else:
                # SER observado = 0; para la gráfica usamos una cota superior.
                ser[i] = 0.0
                upper = min(1.0, 3.0/float(n))
                logser[i] = np.log10(max(upper, 1e-12))

        return ser, logser

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

            ser, logser = self._vectors()
            out_ber[k, :] = ser
            out_log[k, :] = logser

        return nout
