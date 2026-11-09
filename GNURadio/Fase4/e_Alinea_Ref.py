"""
e_Alinea_Ref  -  Alineacion automatica de las secuencias Tx y Rx (Guia 4, paso 9)

El transmisor (otro computador) repite una trama conocida de 8192 bits = 2048 simbolos 16-QAM.
Este bloque regenera esa misma trama en el receptor (parametro 'referencia') y la alinea con
los simbolos decididos por el receptor, de modo que e_BER_Curve compare bit a bit las dos
secuencias con delay_bits = 0.

Entrada 0 : indice de simbolo decidido (0..15) a la salida del Constellation Receiver.
Salida 0  : simbolo recibido ya corregido de la ambiguedad de fase (0/90/180/270 grados).
Salida 1  : simbolo de referencia (Tx) alineado con el anterior.
Salida 2  : desfase encontrado (simbolos, 0..2047)  -> equivale al delay_bits = 4*desfase.
Salida 3  : rotacion aplicada (0..3).
Salida 4  : fraccion de simbolos coincidentes en la ultima ventana (0..1).

Funcionamiento (SIN latencia anadida: cada simbolo sale en cuanto entra, con el desfase y la rotacion que dieron
las ventanas anteriores):
  1. Busqueda: correlacion circular (FFT) de la ventana recibida con la trama de referencia,
     para los 2048 desfases y las 4 rotaciones. Se acepta si coincide >= 30 % de los simbolos
     (el azar da 6,25 %).
  2. Seguimiento: el desfase queda fijo; cada 256 simbolos se reevalua la rotacion (el lazo de
     portadora puede saltar 90 grados cuando el ruido es alto); se cambia solo si otra rotacion
     coincide >= 25 % mas que la actual.
  3. Si 8 ventanas de seguimiento (2048 simbolos) coinciden menos de 25 %, se vuelve a buscar (salto de simbolo).
     La busqueda solo cambia el enganche si encuentra >= 30 % de coincidencia; en un enlace con mucho ruido
     (BER ~ 0,4) el enganche se conserva y los errores se cuentan.
Antes del primer enganche las dos salidas son iguales (no se cuentan errores falsos).
Despues del enganche NUNCA se fuerza la referencia: si el enlace pierde el sincronismo, los
errores resultantes se cuentan como errores reales.
"""
import numpy as np
from gnuradio import gr

W = 2048          # una trama = 2048 simbolos (periodo de la referencia)
TRK = 256         # ventana de seguimiento de la rotacion
NPOBRES = 8       # ventanas de seguimiento pobres (acumuladas) antes de volver a buscar (2048 simbolos)
TABLA = np.array([-3-3j, -3-1j, -3+3j, -3+1j, -1-3j, -1-1j, -1+3j, -1+1j,
                  3-3j, 3-1j, 3+3j, 3+1j, 1-3j, 1-1j, 1+3j, 1+1j]) / np.sqrt(10)


def _mapa(k):
    q = TABLA * (1j ** k)
    return np.array([np.argmin(np.abs(TABLA - v)) for v in q], dtype=np.uint8)


class blk(gr.sync_block):
    def __init__(self, referencia=None, umbral_enganche=0.30, umbral_pobre=0.25):
        gr.sync_block.__init__(self, name='e_Alinea_Ref', in_sig=[np.uint8],
                               out_sig=[np.uint8, np.uint8, np.float32, np.float32, np.float32])
        if referencia is None:
            referencia = [0] * 1024      # solo para que GRC pueda leer la firma del bloque
        bits = np.unpackbits(np.array(referencia, dtype=np.uint8))
        assert len(bits) == 4 * W, 'la referencia debe tener 8192 bits (1024 bytes)'
        self.REF = (bits.reshape(-1, 4) * np.array([8, 4, 2, 1])).sum(1).astype(np.uint8)
        self.MAPAS = [_mapa(k) for k in range(4)]
        self.umbral_enganche = float(umbral_enganche)
        self.umbral_pobre = float(umbral_pobre)
        v = np.arange(16, dtype=np.uint8)[:, None]
        self.FB = np.fft.fft((self.REF[None, :] == v).astype(np.float64), axis=1)
        self.V = v
        self.enganchado = False
        self.s = 0
        self.r = 0
        self.match = 0.0
        self.pobres = 0
        self.buf = np.zeros(0, dtype=np.uint8)
        self.trk = np.zeros(0, dtype=np.uint8)
        self.n_trk = 0
        self.p = 0                                      # indice absoluto de simbolo

    def _buscar(self, x):
        mejor = (-1.0, 0, 0)
        for k in range(4):
            y = self.MAPAS[k][x]
            FA = np.fft.fft((y[None, :] == self.V).astype(np.float64), axis=1)
            c = np.fft.ifft(np.conj(FA) * self.FB, axis=1).real.sum(axis=0)
            s = int(np.argmax(c))
            if c[s] > mejor[0]:
                mejor = (float(c[s]), s, k)
        return mejor[0] / W, mejor[1], mejor[2]

    def _ref_en(self, q0, n):
        """Simbolos de referencia para los indices absolutos q0..q0+n-1."""
        return self.REF[(q0 + np.arange(n) + self.s) % W]

    def _buscar_buffer(self):
        """Busqueda global sobre los ultimos W simbolos recibidos."""
        x = self.buf
        m, s, k = self._buscar(x)
        if m >= self.umbral_enganche:
            inicio = self.p - W                       # indice absoluto del primer simbolo del buffer
            return m, (s - inicio) % W, k
        return m, None, None

    def _seguimiento(self, x):
        """x: ultimos TRK simbolos (ya incluidos en el buffer). Actualiza rotacion / reenganche."""
        q0 = self.p - len(x)
        ref = self._ref_en(q0, len(x))
        coin = [float((self.MAPAS[k][x] == ref).mean()) for k in range(4)]
        mejor = int(np.argmax(coin))
        if coin[mejor] > coin[self.r] + 0.25:
            self.r = mejor
        self.match = coin[self.r]
        if self.match < self.umbral_pobre:
            self.pobres += 1
        else:
            self.pobres = max(0, self.pobres - 2)

    def work(self, input_items, output_items):
        n = len(input_items[0])
        x = input_items[0] & 0x0F
        a = 0
        while a < n:
            b = min(n, a + TRK - len(self.trk))
            seg = x[a:b]
            if self.enganchado:
                output_items[0][a:b] = self.MAPAS[self.r][seg]
                output_items[1][a:b] = self._ref_en(self.p, len(seg))
            else:                                       # antes del primer enganche: sin errores falsos
                output_items[0][a:b] = seg
                output_items[1][a:b] = seg
            self.p += len(seg)
            self.buf = np.concatenate([self.buf, seg])[-W:]
            self.trk = np.concatenate([self.trk, seg])
            if len(self.trk) == TRK:
                if self.enganchado:
                    self._seguimiento(self.trk)
                self.trk = np.zeros(0, dtype=np.uint8)
                self.n_trk += 1
                if len(self.buf) == W and self.n_trk % (W // TRK) == 0 and \
                        (not self.enganchado or self.pobres >= NPOBRES):
                    m, s_abs, k = self._buscar_buffer()
                    if s_abs is not None:
                        self.enganchado, self.s, self.r, self.match, self.pobres = True, s_abs, k, m, 0
                    elif not self.enganchado:
                        self.match = 0.0
            a = b
        output_items[2][:] = self.s
        output_items[3][:] = self.r
        output_items[4][:] = self.match
        return n
