#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Fase 2 A1-GI: enlace 16-QAM Tx-Canal-Rx (inspeccion optica)
# Author: Grupo A1-GI
# Description: 16QAM Tx-Canal-Rx: RRC, canal LOS/EMI/sombra, filtro acoplado, muestreo, AGC, decisor, BER/EVM
# GNU Radio version: 3.10.12.0

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from gnuradio import analog
from gnuradio import blocks
import numpy
from gnuradio import digital
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
import Fase2_A1GI_16QAM_TxCanalRx_ruido_emi as ruido_emi  # embedded python block
import math
import sip
import threading


def snipfcn_captura_auto(self):
    # --- Guion automatico de pruebas y capturas (solo corre si modo_captura == 1) ---
    import os, json, math, traceback
    from PyQt5 import QtCore
    if self.modo_captura:
        self.resize(1900, 1150)
        _dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'capturas')
        os.makedirs(_dir, exist_ok=True)
        _res = {'escenarios': {}, 'barrido_muestreo': [], 'barrido_ber': [], 'alineacion': []}
        _W = {
            'psd_tx_canal': self._psd_tx_canal_win, 'psd_beta': self._psd_beta_win,
            'envolvente': self._envolvente_canal_win, 'const_canal': self._const_canal_win,
            'ojo': self._ojo_filtro_acoplado_win, 'const_rx': self._const_rx_win,
            'simbolos': self._simbolos_tx_rx_win, 'ber': self._ber_num_win, 'evm': self._evm_num_win,
        }
        def _grab(nombre, clave):
            _W[clave].grab().save(os.path.join(_dir, nombre + '.png'))
        def _guardar():
            with open(os.path.join(_dir, 'resultados.json'), 'w') as f:
                json.dump(_res, f, indent=1)
        def _bits():
            return self.ber_promedio.nitems_read(0)
        def _esperar_bits(n):
            n0 = _bits()
            while _bits() - n0 < n:
                yield
        def _esperar_ms(ms):
            t = QtCore.QElapsedTimer(); t.start()
            while t.elapsed() < ms:
                yield
        def _conf(EsN0=30.0, awgn=1, att=0.0, emi=0, isr=0.0, d=7, agc=1, ref=None):
            self.set_EsN0_dB(EsN0); self.set_awgn_on(awgn); self.set_atten_dB(att)
            self.set_emi_on(emi); self.set_emi_isr_dB(isr); self.set_d_muestreo(d); self.set_agc_on(agc)
            if not agc:
                self.rx_agc.set_gain(1.0)
            if ref is not None:
                self.set_retardo_ref(ref)
        def _medir(nombre, k=3, extra=None):
            yield from _esperar_bits(2 * self.ber_win)
            b, e = [], []
            for i in range(k):
                yield from _esperar_bits(self.ber_win)
                b.append(self.probe_ber.level()); e.append(self.probe_evm.level())
            r = dict(ber=sum(b) / k, ber_lecturas=b, evm_pct=sum(e) / k, bits=k * self.ber_win,
                     EsN0_dB=self.EsN0_dB, awgn_on=self.awgn_on, atten_dB=self.atten_dB, emi_on=self.emi_on,
                     emi_isr_dB=self.emi_isr_dB, d_muestreo=self.d_muestreo, agc_on=self.agc_on,
                     retardo_ref=self.retardo_ref, noise_amp=self.noise_amp)
            if extra:
                r.update(extra)
            _res['escenarios'][nombre] = r
            _guardar()
            return r
        def _guion():
            yield from _esperar_ms(3000)
            t0 = QtCore.QElapsedTimer(); t0.start(); n0 = _bits()
            yield from _esperar_ms(2000)
            _res['bits_por_segundo_simulados'] = (_bits() - n0) / (t0.elapsed() / 1000.0)
            # 1) Alineacion de la referencia (canal ideal, muestreo d=7)
            _conf(awgn=0, d=7, agc=1)
            mejor, mejor_ber = 11, 1.0
            for ref in range(6, 17):
                self.set_retardo_ref(ref)
                yield from _esperar_bits(3 * self.ber_win)
                v = self.probe_ber.level()
                _res['alineacion'].append([ref, v])
                if v < mejor_ber:
                    mejor, mejor_ber = ref, v
            _res['retardo_ref_optimo'] = mejor
            _guardar()
            # 2) Canal ideal
            _conf(awgn=0, d=7, agc=1, ref=mejor)
            yield from _medir('ideal', k=3)
            yield from _esperar_ms(1500)
            for c in ['psd_tx_canal', 'psd_beta', 'envolvente', 'const_canal', 'ojo', 'const_rx', 'simbolos', 'ber', 'evm']:
                _grab('ideal_' + c, c)
            self.grab().save(os.path.join(_dir, 'ventana_ideal.png'))
            # 3) Barrido del instante de muestreo (canal ideal)
            for d in range(8):
                mejor_d = None
                for ref in (mejor - 1, mejor, mejor + 1):
                    _conf(awgn=0, d=d, agc=1, ref=ref)
                    r = yield from _medir('muestreo_d%d_ref%d' % (d, ref), k=2)
                    if mejor_d is None or r['ber'] < mejor_d[1]:
                        mejor_d = (ref, r['ber'], r['evm_pct'])
                _res['barrido_muestreo'].append(dict(d=d, ref=mejor_d[0], ber=mejor_d[1], evm_pct=mejor_d[2]))
                _guardar()
                if d in (0, 3, 6):
                    _conf(awgn=0, d=d, agc=1, ref=mejor_d[0])
                    yield from _esperar_bits(2 * self.ber_win)
                    yield from _esperar_ms(1200)
                    _grab('muestreo_d%d_const_rx' % d, 'const_rx')
            # 4) LOS nominal (Es/N0 = 30 dB)
            _conf(EsN0=30, d=7, ref=mejor)
            yield from _medir('los', k=3)
            yield from _esperar_ms(1500)
            for c in ['psd_tx_canal', 'envolvente', 'const_canal', 'ojo', 'const_rx', 'simbolos', 'ber', 'evm']:
                _grab('los_' + c, c)
            # 5) EMI impulsiva (VFD/soldadura) sobre el LOS
            for isr in (-10.0, -5.0, 0.0, 5.0):
                _conf(EsN0=30, emi=1, isr=isr, d=7, ref=mejor)
                yield from _medir('emi_isr%+d' % int(isr), k=5)
            _conf(EsN0=30, emi=1, isr=0.0, d=7, ref=mejor)
            yield from _esperar_bits(2 * self.ber_win)
            yield from _esperar_ms(1500)
            for c in ['psd_tx_canal', 'envolvente', 'const_canal', 'ojo', 'const_rx', 'ber', 'evm']:
                _grab('emi_' + c, c)
            # 5b) AWGN con la MISMA potencia media que EMI(0 dB) + fondo
            Ps = self.E_pulso / self.Sps
            duty = self.emi_rate * self.emi_dur
            P_imp = duty * Ps * 10 ** (0.0 / 10)
            P_bg = self.E_pulso / 10 ** (30.0 / 10)
            Eq = 10 * math.log10(self.E_pulso / (P_bg + P_imp))
            _conf(EsN0=Eq, emi=0, d=7, ref=mejor)
            yield from _medir('awgn_equivalente_emi', k=3, extra=dict(EsN0_equivalente=Eq, duty=duty))
            yield from _esperar_ms(1200)
            _grab('awgn_equiv_const_rx', 'const_rx')
            # 6) Sombra sin AGC (prueba fallida) y con AGC
            for att in (10.0, 15.0, 20.0):
                _conf(EsN0=30, att=att, agc=0, d=7, ref=mejor)
                yield from _medir('sombra%d_sinAGC' % int(att), k=2)
                if att == 10.0:
                    yield from _esperar_ms(1200)
                    _grab('sombra10_sinAGC_const_rx', 'const_rx')
            for att in (10.0, 15.0, 20.0):
                _conf(EsN0=30, att=att, agc=1, d=7, ref=mejor)
                yield from _medir('sombra%d_conAGC' % int(att), k=6 if att == 10.0 else 3)
                yield from _esperar_ms(1200)
                _grab('sombra%d_conAGC_const_rx' % int(att), 'const_rx')
                if att == 15.0:
                    _grab('sombra15_conAGC_const_canal', 'const_canal')
                    _grab('sombra15_conAGC_psd', 'psd_tx_canal')
            # 7) Curva BER vs Eb/N0 medida en GNU Radio (AWGN, sin sombra ni EMI)
            for agc in (1, 0):
                for EbN0 in (0, 2, 4, 6, 8, 10, 12):
                    EsN0 = EbN0 + 10 * math.log10(self.bps)
                    _conf(EsN0=EsN0, agc=agc, d=7, ref=mejor)
                    k = 3 if EbN0 <= 8 else 10
                    r = yield from _medir('curva_agc%d_EbN0_%d' % (agc, EbN0), k=k)
                    _res['barrido_ber'].append(dict(agc=agc, EbN0_dB=EbN0, EsN0_dB=EsN0, ber=r['ber'], evm_pct=r['evm_pct'], bits=r['bits']))
                    _guardar()
            _conf(EsN0=30, d=7, ref=mejor)
            _res['fin'] = True
            _guardar()
        _g = _guion()
        def _paso():
            try:
                next(_g)
            except StopIteration:
                self._timer_captura.stop()
                self.close()
            except Exception:
                self._timer_captura.stop()
                with open(os.path.join(_dir, 'error.txt'), 'w') as f:
                    f.write(traceback.format_exc())
        self._timer_captura = QtCore.QTimer()
        self._timer_captura.timeout.connect(_paso)
        self._timer_captura.start(50)


def snippets_main_after_start(tb):
    snipfcn_captura_auto(tb)

class Fase2_A1GI_16QAM_TxCanalRx(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Fase 2 A1-GI: enlace 16-QAM Tx-Canal-Rx (inspeccion optica)", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Fase 2 A1-GI: enlace 16-QAM Tx-Canal-Rx (inspeccion optica)")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "Fase2_A1GI_16QAM_TxCanalRx")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.Sps = Sps = 8
        self.Rs = Rs = 3000000
        self.tabla_de_verdad_constelacion = tabla_de_verdad_constelacion = [p/math.sqrt(10) for p in (-3-3j, -3-1j, -3+3j, -3+1j, -1-3j, -1-1j, -1+3j, -1+1j, 3-3j, 3-1j, 3+3j, 3+1j, 1-3j, 1-1j, 1+3j, 1+1j)]
        self.span = span = 11
        self.samp_rate = samp_rate = Rs*Sps
        self.beta = beta = 0.35
        self.rrc_tx_taps = rrc_tx_taps = firdes.root_raised_cosine(Sps, samp_rate,Rs, beta, (span*Sps))
        self.M = M = len(tabla_de_verdad_constelacion)
        self.emi_isr_dB = emi_isr_dB = 0
        self.bps = bps = int(math.log(M,2))
        self.awgn_on = awgn_on = 1
        self.atten_dB = atten_dB = 0
        self.EsN0_dB = EsN0_dB = 30
        self.E_pulso = E_pulso = sum(t*t for t in rrc_tx_taps)
        self.rrc_rx_taps = rrc_rx_taps = [t/E_pulso for t in rrc_tx_taps]
        self.retardo_ref = retardo_ref = 11
        self.ref_agc = ref_agc = sum(abs(p) for p in tabla_de_verdad_constelacion)/M
        self.noise_amp = noise_amp = awgn_on*math.sqrt(E_pulso/10**(EsN0_dB/10))
        self.modo_captura = modo_captura = 0
        self.g_sombra = g_sombra = 10**(-atten_dB/20)
        self.emi_rate = emi_rate = 16e3
        self.emi_on = emi_on = 0
        self.emi_dur = emi_dur = 1e-6
        self.emi_amp = emi_amp = math.sqrt(E_pulso/Sps*10**(emi_isr_dB/10))
        self.d_muestreo = d_muestreo = 7
        self.constelacion_16qam = constelacion_16qam = digital.constellation_calcdist(tabla_de_verdad_constelacion, [], 4, 1, digital.constellation.NO_NORMALIZATION)
        self.ber_win = ber_win = 200000
        self.agc_rate = agc_rate = 1e-3
        self.agc_on = agc_on = 1
        self.Rb = Rb = Rs*bps

        ##################################################
        # Blocks
        ##################################################

        self._retardo_ref_range = qtgui.Range(0, 30, 1, 11, 200)
        self._retardo_ref_win = qtgui.RangeWidget(self._retardo_ref_range, self.set_retardo_ref, "Retardo referencia BER [simbolos]", "counter_slider", int, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._retardo_ref_win, 0, 4, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(4, 5):
            self.top_grid_layout.setColumnStretch(c, 1)
        _emi_on_check_box = Qt.QCheckBox("EMI impulsiva (VFD/soldadura)")
        self._emi_on_choices = {True: 1, False: 0}
        self._emi_on_choices_inv = dict((v,k) for k,v in self._emi_on_choices.items())
        self._emi_on_callback = lambda i: Qt.QMetaObject.invokeMethod(_emi_on_check_box, "setChecked", Qt.Q_ARG("bool", self._emi_on_choices_inv[i]))
        self._emi_on_callback(self.emi_on)
        _emi_on_check_box.stateChanged.connect(lambda i: self.set_emi_on(self._emi_on_choices[bool(i)]))
        self.top_grid_layout.addWidget(_emi_on_check_box, 1, 2, 1, 2)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(2, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._d_muestreo_range = qtgui.Range(0, 7, 1, 7, 200)
        self._d_muestreo_win = qtgui.RangeWidget(self._d_muestreo_range, self.set_d_muestreo, "Retardo de muestreo d [muestras]", "counter_slider", int, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._d_muestreo_win, 0, 3, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(3, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        _agc_on_check_box = Qt.QCheckBox("AGC en el receptor")
        self._agc_on_choices = {True: 1, False: 0}
        self._agc_on_choices_inv = dict((v,k) for k,v in self._agc_on_choices.items())
        self._agc_on_callback = lambda i: Qt.QMetaObject.invokeMethod(_agc_on_check_box, "setChecked", Qt.Q_ARG("bool", self._agc_on_choices_inv[i]))
        self._agc_on_callback(self.agc_on)
        _agc_on_check_box.stateChanged.connect(lambda i: self.set_agc_on(self._agc_on_choices[bool(i)]))
        self.top_grid_layout.addWidget(_agc_on_check_box, 1, 4, 1, 2)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(4, 6):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.throttle_0 = blocks.throttle( gr.sizeof_gr_complex*1, samp_rate, True, 0 if "auto" == "auto" else max( int(float(0.1) * samp_rate) if "auto" == "time" else int(0.1), 1) )
        self.simbolos_tx_rx = qtgui.time_sink_f(
            48, #size
            Rs, #samp_rate
            "Simbolos transmitidos vs recuperados (indice 0-15)", #name
            2, #number of inputs
            None # parent
        )
        self.simbolos_tx_rx.set_update_time(0.1)
        self.simbolos_tx_rx.set_y_axis(-1, 16)

        self.simbolos_tx_rx.set_y_label('Indice de simbolo', "")

        self.simbolos_tx_rx.enable_tags(False)
        self.simbolos_tx_rx.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.simbolos_tx_rx.enable_autoscale(False)
        self.simbolos_tx_rx.enable_grid(True)
        self.simbolos_tx_rx.enable_axis_labels(True)
        self.simbolos_tx_rx.enable_control_panel(False)
        self.simbolos_tx_rx.enable_stem_plot(False)


        labels = ['Tx (referencia)', 'Rx (decisor)', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [2, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 0, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, 0, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                self.simbolos_tx_rx.set_line_label(i, "Data {0}".format(i))
            else:
                self.simbolos_tx_rx.set_line_label(i, labels[i])
            self.simbolos_tx_rx.set_line_width(i, widths[i])
            self.simbolos_tx_rx.set_line_color(i, colors[i])
            self.simbolos_tx_rx.set_line_style(i, styles[i])
            self.simbolos_tx_rx.set_line_marker(i, markers[i])
            self.simbolos_tx_rx.set_line_alpha(i, alphas[i])

        self._simbolos_tx_rx_win = sip.wrapinstance(self.simbolos_tx_rx.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._simbolos_tx_rx_win, 10, 0, 3, 2)
        for r in range(10, 13):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.rx_punto_ideal = digital.chunks_to_symbols_bc(tabla_de_verdad_constelacion, 1)
        self.rx_muestreo = blocks.keep_one_in_n(gr.sizeof_gr_complex*1, Sps)
        self.rx_float = blocks.uchar_to_float()
        self.rx_filtro_acoplado = filter.fir_filter_ccf(1, rrc_rx_taps)
        self.rx_filtro_acoplado.declare_sample_delay(0)
        self.rx_decisor = digital.constellation_decoder_cb(constelacion_16qam)
        self.rx_ajuste_muestreo = blocks.delay(gr.sizeof_gr_complex*1, d_muestreo)
        self.rx_agc = analog.agc_cc((agc_rate*agc_on), ref_agc, 1.0, 100)
        self.ruido_emi = ruido_emi.blk(samp_rate=samp_rate, rate=emi_rate, dur=emi_dur, amp=emi_amp, enable=emi_on)
        self.ruido_awgn = analog.noise_source_c(analog.GR_GAUSSIAN, noise_amp, 0)
        self.rrc_tx = filter.interp_fir_filter_ccf(Sps, rrc_tx_taps)
        self.rrc_tx.declare_sample_delay(0)
        self.rrc_b050 = filter.interp_fir_filter_ccf(Sps, firdes.root_raised_cosine(Sps, samp_rate, Rs, 0.50, span*Sps))
        self.rrc_b050.declare_sample_delay(0)
        self.rrc_b020 = filter.interp_fir_filter_ccf(Sps, firdes.root_raised_cosine(Sps, samp_rate, Rs, 0.20, span*Sps))
        self.rrc_b020.declare_sample_delay(0)
        self.ref_retardo = blocks.delay(gr.sizeof_char*1, retardo_ref)
        self.ref_float = blocks.uchar_to_float()
        self.psd_tx_canal = qtgui.freq_sink_c(
            2048, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "PSD: salida RRC Tx vs salida del canal", #name
            2,
            None # parent
        )
        self.psd_tx_canal.set_update_time(0.1)
        self.psd_tx_canal.set_y_axis((-120), (-10))
        self.psd_tx_canal.set_y_label('Relative Gain', 'dB')
        self.psd_tx_canal.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.psd_tx_canal.enable_autoscale(False)
        self.psd_tx_canal.enable_grid(True)
        self.psd_tx_canal.set_fft_average(0.1)
        self.psd_tx_canal.enable_axis_labels(True)
        self.psd_tx_canal.enable_control_panel(False)
        self.psd_tx_canal.set_fft_window_normalized(False)



        labels = ['Tx (RRC)', 'Salida del canal', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(2):
            if len(labels[i]) == 0:
                self.psd_tx_canal.set_line_label(i, "Data {0}".format(i))
            else:
                self.psd_tx_canal.set_line_label(i, labels[i])
            self.psd_tx_canal.set_line_width(i, widths[i])
            self.psd_tx_canal.set_line_color(i, colors[i])
            self.psd_tx_canal.set_line_alpha(i, alphas[i])

        self._psd_tx_canal_win = sip.wrapinstance(self.psd_tx_canal.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._psd_tx_canal_win, 2, 0, 4, 2)
        for r in range(2, 6):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.psd_beta = qtgui.freq_sink_c(
            2048, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "PSD Tx para distintos roll-off", #name
            3,
            None # parent
        )
        self.psd_beta.set_update_time(0.1)
        self.psd_beta.set_y_axis((-120), (-10))
        self.psd_beta.set_y_label('Relative Gain', 'dB')
        self.psd_beta.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.psd_beta.enable_autoscale(False)
        self.psd_beta.enable_grid(True)
        self.psd_beta.set_fft_average(0.1)
        self.psd_beta.enable_axis_labels(True)
        self.psd_beta.enable_control_panel(False)
        self.psd_beta.set_fft_window_normalized(False)



        labels = ['beta = 0.20', 'beta = 0.35 (diseno)', 'beta = 0.50', '', '',
            '', '', '', '', '']
        widths = [1, 2, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["green", "blue", "red", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(3):
            if len(labels[i]) == 0:
                self.psd_beta.set_line_label(i, "Data {0}".format(i))
            else:
                self.psd_beta.set_line_label(i, labels[i])
            self.psd_beta.set_line_width(i, widths[i])
            self.psd_beta.set_line_color(i, colors[i])
            self.psd_beta.set_line_alpha(i, alphas[i])

        self._psd_beta_win = sip.wrapinstance(self.psd_beta.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._psd_beta_win, 2, 2, 4, 2)
        for r in range(2, 6):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(2, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.probe_evm = blocks.probe_signal_f()
        self.probe_ber = blocks.probe_signal_f()
        self.ojo_filtro_acoplado = qtgui.eye_sink_c(
            1024, #size
            samp_rate, #samp_rate
            1, #number of inputs
            None
        )
        self.ojo_filtro_acoplado.set_update_time(0.1)
        self.ojo_filtro_acoplado.set_samp_per_symbol(Sps)
        self.ojo_filtro_acoplado.set_y_axis(-1.5, 1.5)

        self.ojo_filtro_acoplado.set_y_label('Amplitud', "")

        self.ojo_filtro_acoplado.enable_tags(False)
        self.ojo_filtro_acoplado.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.ojo_filtro_acoplado.enable_autoscale(False)
        self.ojo_filtro_acoplado.enable_grid(True)
        self.ojo_filtro_acoplado.enable_axis_labels(True)
        self.ojo_filtro_acoplado.enable_control_panel(False)


        labels = ['I (filtro acoplado)', 'Q (filtro acoplado)', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'blue', 'blue', 'blue', 'blue',
            'blue', 'blue', 'blue', 'blue', 'blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                if (i % 2 == 0):
                    self.ojo_filtro_acoplado.set_line_label(i, "Eye [Re{{Data {0}}}]".format(round(i/2)))
                else:
                    self.ojo_filtro_acoplado.set_line_label(i, "Eye [Im{{Data {0}}}]".format(round((i-1)/2)))
            else:
                self.ojo_filtro_acoplado.set_line_label(i, labels[i])
            self.ojo_filtro_acoplado.set_line_width(i, widths[i])
            self.ojo_filtro_acoplado.set_line_color(i, colors[i])
            self.ojo_filtro_acoplado.set_line_style(i, styles[i])
            self.ojo_filtro_acoplado.set_line_marker(i, markers[i])
            self.ojo_filtro_acoplado.set_line_alpha(i, alphas[i])

        self._ojo_filtro_acoplado_win = sip.wrapinstance(self.ojo_filtro_acoplado.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._ojo_filtro_acoplado_win, 6, 2, 4, 2)
        for r in range(6, 10):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(2, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.mapeo_16qam = digital.chunks_to_symbols_bc(tabla_de_verdad_constelacion, 1)
        self.fuente_bits = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 131072))), True)
        self.evm_raiz = blocks.transcendental('sqrt', "float")
        self.evm_promedio = blocks.moving_average_ff(20000, (1/20000), 4000, 1)
        self.evm_pct = blocks.multiply_const_ff(100)
        self.evm_num = qtgui.number_sink(
            gr.sizeof_float,
            0,
            qtgui.NUM_GRAPH_HORIZ,
            1,
            None # parent
        )
        self.evm_num.set_update_time(0.2)
        self.evm_num.set_title("EVM rms")

        labels = ['EVM', '', '', '', '',
            '', '', '', '', '']
        units = ['%', '', '', '', '',
            '', '', '', '', '']
        colors = [("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"),
            ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black")]
        factor = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]

        for i in range(1):
            self.evm_num.set_min(i, 0)
            self.evm_num.set_max(i, 40)
            self.evm_num.set_color(i, colors[i][0], colors[i][1])
            if len(labels[i]) == 0:
                self.evm_num.set_label(i, "Data {0}".format(i))
            else:
                self.evm_num.set_label(i, labels[i])
            self.evm_num.set_unit(i, units[i])
            self.evm_num.set_factor(i, factor[i])

        self.evm_num.enable_autoscale(False)
        self._evm_num_win = sip.wrapinstance(self.evm_num.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._evm_num_win, 10, 4, 3, 2)
        for r in range(10, 13):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(4, 6):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.evm_mag2 = blocks.complex_to_mag_squared(1)
        self.evm_error = blocks.sub_cc(1)
        self.envolvente_canal = qtgui.time_sink_c(
            4096, #size
            samp_rate, #samp_rate
            "Envolvente compleja a la salida del canal", #name
            1, #number of inputs
            None # parent
        )
        self.envolvente_canal.set_update_time(0.1)
        self.envolvente_canal.set_y_axis(-3, 3)

        self.envolvente_canal.set_y_label('Amplitud', "")

        self.envolvente_canal.enable_tags(False)
        self.envolvente_canal.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.envolvente_canal.enable_autoscale(False)
        self.envolvente_canal.enable_grid(True)
        self.envolvente_canal.enable_axis_labels(True)
        self.envolvente_canal.enable_control_panel(False)
        self.envolvente_canal.enable_stem_plot(False)


        labels = ['I', 'Q', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                if (i % 2 == 0):
                    self.envolvente_canal.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.envolvente_canal.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.envolvente_canal.set_line_label(i, labels[i])
            self.envolvente_canal.set_line_width(i, widths[i])
            self.envolvente_canal.set_line_color(i, colors[i])
            self.envolvente_canal.set_line_style(i, styles[i])
            self.envolvente_canal.set_line_marker(i, markers[i])
            self.envolvente_canal.set_line_alpha(i, alphas[i])

        self._envolvente_canal_win = sip.wrapinstance(self.envolvente_canal.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._envolvente_canal_win, 2, 4, 4, 2)
        for r in range(2, 6):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(4, 6):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._emi_isr_dB_range = qtgui.Range(-15, 10, 1, 0, 200)
        self._emi_isr_dB_win = qtgui.RangeWidget(self._emi_isr_dB_range, self.set_emi_isr_dB, "EMI: potencia rafaga/senal [dB]", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._emi_isr_dB_win, 0, 2, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(2, 3):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.const_rx = qtgui.const_sink_c(
            2048, #size
            "Constelacion recuperada (1 muestra/simbolo)", #name
            1, #number of inputs
            None # parent
        )
        self.const_rx.set_update_time(0.1)
        self.const_rx.set_y_axis((-1.5), 1.5)
        self.const_rx.set_x_axis((-1.5), 1.5)
        self.const_rx.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, "")
        self.const_rx.enable_autoscale(False)
        self.const_rx.enable_grid(True)
        self.const_rx.enable_axis_labels(True)

        self.const_rx.disable_legend()

        labels = ["Rx", '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        styles = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        markers = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.const_rx.set_line_label(i, "Data {0}".format(i))
            else:
                self.const_rx.set_line_label(i, labels[i])
            self.const_rx.set_line_width(i, widths[i])
            self.const_rx.set_line_color(i, colors[i])
            self.const_rx.set_line_style(i, styles[i])
            self.const_rx.set_line_marker(i, markers[i])
            self.const_rx.set_line_alpha(i, alphas[i])

        self._const_rx_win = sip.wrapinstance(self.const_rx.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._const_rx_win, 6, 4, 4, 2)
        for r in range(6, 10):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(4, 6):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.const_canal = qtgui.const_sink_c(
            4096, #size
            "Plano I-Q sobremuestreado (salida del canal)", #name
            1, #number of inputs
            None # parent
        )
        self.const_canal.set_update_time(0.1)
        self.const_canal.set_y_axis((-2), 2)
        self.const_canal.set_x_axis((-2), 2)
        self.const_canal.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, "")
        self.const_canal.enable_autoscale(False)
        self.const_canal.enable_grid(True)
        self.const_canal.enable_axis_labels(True)

        self.const_canal.disable_legend()

        labels = ["Canal", '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        styles = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        markers = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.const_canal.set_line_label(i, "Data {0}".format(i))
            else:
                self.const_canal.set_line_label(i, labels[i])
            self.const_canal.set_line_width(i, widths[i])
            self.const_canal.set_line_color(i, colors[i])
            self.const_canal.set_line_style(i, styles[i])
            self.const_canal.set_line_marker(i, markers[i])
            self.const_canal.set_line_alpha(i, alphas[i])

        self._const_canal_win = sip.wrapinstance(self.const_canal.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._const_canal_win, 6, 0, 4, 2)
        for r in range(6, 10):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.canal_suma = blocks.add_vcc(1)
        self.canal_sombra = blocks.multiply_const_cc(g_sombra)
        self.bits_a_simbolos = blocks.repack_bits_bb(1, bps, "", False, gr.GR_MSB_FIRST)
        self._beta_range = qtgui.Range(0.1, 1.0, 0.05, 0.35, 200)
        self._beta_win = qtgui.RangeWidget(self._beta_range, self.set_beta, "Roll-off beta", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._beta_win, 0, 5, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(5, 6):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.ber_xor = blocks.xor_bb()
        self.ber_unpack = blocks.unpack_k_bits_bb(bps)
        self.ber_promedio = blocks.moving_average_ff(ber_win, (1.0/ber_win), 4000, 1)
        self.ber_num = qtgui.number_sink(
            gr.sizeof_float,
            0,
            qtgui.NUM_GRAPH_HORIZ,
            1,
            None # parent
        )
        self.ber_num.set_update_time(0.2)
        self.ber_num.set_title("BER medida")

        labels = ['BER', '', '', '', '',
            '', '', '', '', '']
        units = ['', '', '', '', '',
            '', '', '', '', '']
        colors = [("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"),
            ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black")]
        factor = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]

        for i in range(1):
            self.ber_num.set_min(i, 0)
            self.ber_num.set_max(i, 0.5)
            self.ber_num.set_color(i, colors[i][0], colors[i][1])
            if len(labels[i]) == 0:
                self.ber_num.set_label(i, "Data {0}".format(i))
            else:
                self.ber_num.set_label(i, labels[i])
            self.ber_num.set_unit(i, units[i])
            self.ber_num.set_factor(i, factor[i])

        self.ber_num.enable_autoscale(False)
        self._ber_num_win = sip.wrapinstance(self.ber_num.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._ber_num_win, 10, 2, 3, 2)
        for r in range(10, 13):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(2, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.ber_float = blocks.uchar_to_float()
        _awgn_on_check_box = Qt.QCheckBox("AWGN de fondo")
        self._awgn_on_choices = {True: 1, False: 0}
        self._awgn_on_choices_inv = dict((v,k) for k,v in self._awgn_on_choices.items())
        self._awgn_on_callback = lambda i: Qt.QMetaObject.invokeMethod(_awgn_on_check_box, "setChecked", Qt.Q_ARG("bool", self._awgn_on_choices_inv[i]))
        self._awgn_on_callback(self.awgn_on)
        _awgn_on_check_box.stateChanged.connect(lambda i: self.set_awgn_on(self._awgn_on_choices[bool(i)]))
        self.top_grid_layout.addWidget(_awgn_on_check_box, 1, 0, 1, 2)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._atten_dB_range = qtgui.Range(0, 25, 1, 0, 200)
        self._atten_dB_win = qtgui.RangeWidget(self._atten_dB_range, self.set_atten_dB, "Atenuacion por sombra [dB]", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._atten_dB_win, 0, 1, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._EsN0_dB_range = qtgui.Range(0, 45, 0.5, 30, 200)
        self._EsN0_dB_win = qtgui.RangeWidget(self._EsN0_dB_range, self.set_EsN0_dB, "Es/N0 LOS [dB]", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._EsN0_dB_win, 0, 0, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.ber_float, 0), (self.ber_promedio, 0))
        self.connect((self.ber_promedio, 0), (self.ber_num, 0))
        self.connect((self.ber_promedio, 0), (self.probe_ber, 0))
        self.connect((self.ber_unpack, 0), (self.ber_float, 0))
        self.connect((self.ber_xor, 0), (self.ber_unpack, 0))
        self.connect((self.bits_a_simbolos, 0), (self.mapeo_16qam, 0))
        self.connect((self.bits_a_simbolos, 0), (self.ref_retardo, 0))
        self.connect((self.canal_sombra, 0), (self.canal_suma, 0))
        self.connect((self.canal_suma, 0), (self.const_canal, 0))
        self.connect((self.canal_suma, 0), (self.envolvente_canal, 0))
        self.connect((self.canal_suma, 0), (self.psd_tx_canal, 1))
        self.connect((self.canal_suma, 0), (self.rx_filtro_acoplado, 0))
        self.connect((self.evm_error, 0), (self.evm_mag2, 0))
        self.connect((self.evm_mag2, 0), (self.evm_promedio, 0))
        self.connect((self.evm_pct, 0), (self.evm_num, 0))
        self.connect((self.evm_pct, 0), (self.probe_evm, 0))
        self.connect((self.evm_promedio, 0), (self.evm_raiz, 0))
        self.connect((self.evm_raiz, 0), (self.evm_pct, 0))
        self.connect((self.fuente_bits, 0), (self.bits_a_simbolos, 0))
        self.connect((self.mapeo_16qam, 0), (self.rrc_b020, 0))
        self.connect((self.mapeo_16qam, 0), (self.rrc_b050, 0))
        self.connect((self.mapeo_16qam, 0), (self.rrc_tx, 0))
        self.connect((self.ref_float, 0), (self.simbolos_tx_rx, 0))
        self.connect((self.ref_retardo, 0), (self.ber_xor, 0))
        self.connect((self.ref_retardo, 0), (self.ref_float, 0))
        self.connect((self.rrc_b020, 0), (self.psd_beta, 0))
        self.connect((self.rrc_b050, 0), (self.psd_beta, 2))
        self.connect((self.rrc_tx, 0), (self.throttle_0, 0))
        self.connect((self.ruido_awgn, 0), (self.canal_suma, 1))
        self.connect((self.ruido_emi, 0), (self.canal_suma, 2))
        self.connect((self.rx_agc, 0), (self.const_rx, 0))
        self.connect((self.rx_agc, 0), (self.evm_error, 0))
        self.connect((self.rx_agc, 0), (self.rx_decisor, 0))
        self.connect((self.rx_ajuste_muestreo, 0), (self.rx_muestreo, 0))
        self.connect((self.rx_decisor, 0), (self.ber_xor, 1))
        self.connect((self.rx_decisor, 0), (self.rx_float, 0))
        self.connect((self.rx_decisor, 0), (self.rx_punto_ideal, 0))
        self.connect((self.rx_filtro_acoplado, 0), (self.ojo_filtro_acoplado, 0))
        self.connect((self.rx_filtro_acoplado, 0), (self.rx_ajuste_muestreo, 0))
        self.connect((self.rx_float, 0), (self.simbolos_tx_rx, 1))
        self.connect((self.rx_muestreo, 0), (self.rx_agc, 0))
        self.connect((self.rx_punto_ideal, 0), (self.evm_error, 1))
        self.connect((self.throttle_0, 0), (self.canal_sombra, 0))
        self.connect((self.throttle_0, 0), (self.psd_beta, 1))
        self.connect((self.throttle_0, 0), (self.psd_tx_canal, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "Fase2_A1GI_16QAM_TxCanalRx")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_Sps(self):
        return self.Sps

    def set_Sps(self, Sps):
        self.Sps = Sps
        self.set_samp_rate(self.Rs*self.Sps)
        self.set_rrc_tx_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, self.beta, (self.span*self.Sps)))
        self.set_emi_amp(math.sqrt(self.E_pulso/self.Sps*10**(self.emi_isr_dB/10)))
        self.rrc_b020.set_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, 0.20, self.span*self.Sps))
        self.rrc_b050.set_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, 0.50, self.span*self.Sps))
        self.rx_muestreo.set_n(self.Sps)
        self.ojo_filtro_acoplado.set_samp_per_symbol(self.Sps)

    def get_Rs(self):
        return self.Rs

    def set_Rs(self, Rs):
        self.Rs = Rs
        self.set_samp_rate(self.Rs*self.Sps)
        self.set_Rb(self.Rs*self.bps)
        self.set_rrc_tx_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, self.beta, (self.span*self.Sps)))
        self.rrc_b020.set_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, 0.20, self.span*self.Sps))
        self.rrc_b050.set_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, 0.50, self.span*self.Sps))
        self.simbolos_tx_rx.set_samp_rate(self.Rs)

    def get_tabla_de_verdad_constelacion(self):
        return self.tabla_de_verdad_constelacion

    def set_tabla_de_verdad_constelacion(self, tabla_de_verdad_constelacion):
        self.tabla_de_verdad_constelacion = tabla_de_verdad_constelacion
        self.set_M(len(self.tabla_de_verdad_constelacion))
        self.set_constelacion_16qam(digital.constellation_calcdist(self.tabla_de_verdad_constelacion, [], 4, 1, digital.constellation.NO_NORMALIZATION))
        self.set_ref_agc(sum(abs(p) for p in self.tabla_de_verdad_constelacion)/self.M)
        self.mapeo_16qam.set_symbol_table(self.tabla_de_verdad_constelacion)
        self.rx_punto_ideal.set_symbol_table(self.tabla_de_verdad_constelacion)

    def get_span(self):
        return self.span

    def set_span(self, span):
        self.span = span
        self.set_rrc_tx_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, self.beta, (self.span*self.Sps)))
        self.rrc_b020.set_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, 0.20, self.span*self.Sps))
        self.rrc_b050.set_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, 0.50, self.span*self.Sps))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_rrc_tx_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, self.beta, (self.span*self.Sps)))
        self.throttle_0.set_sample_rate(self.samp_rate)
        self.rrc_b020.set_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, 0.20, self.span*self.Sps))
        self.rrc_b050.set_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, 0.50, self.span*self.Sps))
        self.ruido_emi.samp_rate = self.samp_rate
        self.psd_tx_canal.set_frequency_range(0, self.samp_rate)
        self.psd_beta.set_frequency_range(0, self.samp_rate)
        self.envolvente_canal.set_samp_rate(self.samp_rate)
        self.ojo_filtro_acoplado.set_samp_rate(self.samp_rate)

    def get_beta(self):
        return self.beta

    def set_beta(self, beta):
        self.beta = beta
        self.set_rrc_tx_taps(firdes.root_raised_cosine(self.Sps, self.samp_rate, self.Rs, self.beta, (self.span*self.Sps)))

    def get_rrc_tx_taps(self):
        return self.rrc_tx_taps

    def set_rrc_tx_taps(self, rrc_tx_taps):
        self.rrc_tx_taps = rrc_tx_taps
        self.set_E_pulso(sum(t*t for t in self.rrc_tx_taps))
        self.set_rrc_rx_taps([t/self.E_pulso for t in self.rrc_tx_taps])
        self.rrc_tx.set_taps(self.rrc_tx_taps)

    def get_M(self):
        return self.M

    def set_M(self, M):
        self.M = M
        self.set_bps(int(math.log(self.M,2)))
        self.set_ref_agc(sum(abs(p) for p in self.tabla_de_verdad_constelacion)/self.M)

    def get_emi_isr_dB(self):
        return self.emi_isr_dB

    def set_emi_isr_dB(self, emi_isr_dB):
        self.emi_isr_dB = emi_isr_dB
        self.set_emi_amp(math.sqrt(self.E_pulso/self.Sps*10**(self.emi_isr_dB/10)))

    def get_bps(self):
        return self.bps

    def set_bps(self, bps):
        self.bps = bps
        self.set_Rb(self.Rs*self.bps)
        self.bits_a_simbolos.set_k_and_l(1,self.bps)

    def get_awgn_on(self):
        return self.awgn_on

    def set_awgn_on(self, awgn_on):
        self.awgn_on = awgn_on
        self.set_noise_amp(self.awgn_on*math.sqrt(self.E_pulso/10**(self.EsN0_dB/10)))
        self._awgn_on_callback(self.awgn_on)

    def get_atten_dB(self):
        return self.atten_dB

    def set_atten_dB(self, atten_dB):
        self.atten_dB = atten_dB
        self.set_g_sombra(10**(-self.atten_dB/20))

    def get_EsN0_dB(self):
        return self.EsN0_dB

    def set_EsN0_dB(self, EsN0_dB):
        self.EsN0_dB = EsN0_dB
        self.set_noise_amp(self.awgn_on*math.sqrt(self.E_pulso/10**(self.EsN0_dB/10)))

    def get_E_pulso(self):
        return self.E_pulso

    def set_E_pulso(self, E_pulso):
        self.E_pulso = E_pulso
        self.set_rrc_rx_taps([t/self.E_pulso for t in self.rrc_tx_taps])
        self.set_noise_amp(self.awgn_on*math.sqrt(self.E_pulso/10**(self.EsN0_dB/10)))
        self.set_emi_amp(math.sqrt(self.E_pulso/self.Sps*10**(self.emi_isr_dB/10)))

    def get_rrc_rx_taps(self):
        return self.rrc_rx_taps

    def set_rrc_rx_taps(self, rrc_rx_taps):
        self.rrc_rx_taps = rrc_rx_taps
        self.rx_filtro_acoplado.set_taps(self.rrc_rx_taps)

    def get_retardo_ref(self):
        return self.retardo_ref

    def set_retardo_ref(self, retardo_ref):
        self.retardo_ref = retardo_ref
        self.ref_retardo.set_dly(int(self.retardo_ref))

    def get_ref_agc(self):
        return self.ref_agc

    def set_ref_agc(self, ref_agc):
        self.ref_agc = ref_agc
        self.rx_agc.set_reference(self.ref_agc)

    def get_noise_amp(self):
        return self.noise_amp

    def set_noise_amp(self, noise_amp):
        self.noise_amp = noise_amp
        self.ruido_awgn.set_amplitude(self.noise_amp)

    def get_modo_captura(self):
        return self.modo_captura

    def set_modo_captura(self, modo_captura):
        self.modo_captura = modo_captura

    def get_g_sombra(self):
        return self.g_sombra

    def set_g_sombra(self, g_sombra):
        self.g_sombra = g_sombra
        self.canal_sombra.set_k(self.g_sombra)

    def get_emi_rate(self):
        return self.emi_rate

    def set_emi_rate(self, emi_rate):
        self.emi_rate = emi_rate
        self.ruido_emi.rate = self.emi_rate

    def get_emi_on(self):
        return self.emi_on

    def set_emi_on(self, emi_on):
        self.emi_on = emi_on
        self._emi_on_callback(self.emi_on)
        self.ruido_emi.enable = self.emi_on

    def get_emi_dur(self):
        return self.emi_dur

    def set_emi_dur(self, emi_dur):
        self.emi_dur = emi_dur
        self.ruido_emi.dur = self.emi_dur

    def get_emi_amp(self):
        return self.emi_amp

    def set_emi_amp(self, emi_amp):
        self.emi_amp = emi_amp
        self.ruido_emi.amp = self.emi_amp

    def get_d_muestreo(self):
        return self.d_muestreo

    def set_d_muestreo(self, d_muestreo):
        self.d_muestreo = d_muestreo
        self.rx_ajuste_muestreo.set_dly(int(self.d_muestreo))

    def get_constelacion_16qam(self):
        return self.constelacion_16qam

    def set_constelacion_16qam(self, constelacion_16qam):
        self.constelacion_16qam = constelacion_16qam
        self.rx_decisor.set_constellation(self.constelacion_16qam)

    def get_ber_win(self):
        return self.ber_win

    def set_ber_win(self, ber_win):
        self.ber_win = ber_win
        self.ber_promedio.set_length_and_scale(self.ber_win, (1.0/self.ber_win))

    def get_agc_rate(self):
        return self.agc_rate

    def set_agc_rate(self, agc_rate):
        self.agc_rate = agc_rate
        self.rx_agc.set_rate((self.agc_rate*self.agc_on))

    def get_agc_on(self):
        return self.agc_on

    def set_agc_on(self, agc_on):
        self.agc_on = agc_on
        self._agc_on_callback(self.agc_on)
        self.rx_agc.set_rate((self.agc_rate*self.agc_on))

    def get_Rb(self):
        return self.Rb

    def set_Rb(self, Rb):
        self.Rb = Rb




def main(top_block_cls=Fase2_A1GI_16QAM_TxCanalRx, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()
    tb.flowgraph_started.set()
    snippets_main_after_start(tb)
    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
