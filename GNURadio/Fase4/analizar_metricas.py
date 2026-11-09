"""Analiza metricas_sistema_GrupoA1-GI.csv (Guia 4A): promedio y mediana por Eb/N0, penalizacion vs teoria,
SNR medida y dispersion I/Q. Solo necesita numpy (matplotlib opcional para la grafica).
Uso: python3 analizar_metricas.py [archivo.csv]"""
import sys, csv, math
import numpy as np
from pathlib import Path
ruta = Path(sys.argv[1] if len(sys.argv) > 1 else 'metricas_sistema_GrupoA1-GI.csv')
filas = list(csv.DictReader(open(ruta, encoding='utf-8')))
ber_t = lambda e: (3/8)*math.erfc(math.sqrt(0.4*10**(e/10)))
ser_t = lambda e: 1-(1-0.75*math.erfc(math.sqrt(0.4*10**(e/10))))**2
niveles = sorted({float(f['ebn0_db']) for f in filas})
def col(e, c): return np.array([float(f[c]) for f in filas if float(f['ebn0_db']) == e])
tab = []
print(f"{'Eb/N0':>6} {'n':>4} {'BER_prom':>10} {'BER_med':>10} {'BER_teo':>10} {'SER_prom':>10} {'SER_med':>10} {'SER_teo':>10} {'SNR_med':>8} {'disp':>6}")
for e in niveles:
    b, s = col(e, 'measured_ber'), col(e, 'measured_ser')
    r = dict(e=e, n=len(b), bp=b.mean(), bm=np.median(b), bt=ber_t(e), sp=s.mean(), sm=np.median(s), st=ser_t(e),
             snr=np.nanmean(col(e, 'measured_snr_db')), disp=np.nanmean(col(e, 'iq_dispersion_rms')))
    tab.append(r)
    print(f"{e:6.1f} {r['n']:4d} {r['bp']:10.3e} {r['bm']:10.3e} {r['bt']:10.3e} {r['sp']:10.3e} {r['sm']:10.3e} {r['st']:10.3e} {r['snr']:8.2f} {r['disp']:6.3f}")
def cruce(x, y, nivel):
    ly = np.log10(np.maximum(np.asarray(y, float), 1e-12))
    for i in range(len(x)-1):
        if ly[i] >= np.log10(nivel) > ly[i+1]: return x[i]+(ly[i]-np.log10(nivel))/(ly[i]-ly[i+1])*(x[i+1]-x[i])
xx = np.linspace(0, 14, 1400); x = [r['e'] for r in tab]
print()
for nom, f, cp, cm in (('BER', ber_t, 'bp', 'bm'), ('SER', ser_t, 'sp', 'sm')):
    for niv in (1e-2, 1e-3):
        t = cruce(xx, [f(e) for e in xx], niv)
        for est, c in (('promedio', cp), ('mediana', cm)):
            m = cruce(x, [r[c] for r in tab], niv)
            print(f'{nom} = {niv:g} ({est}): teoria {t:.2f} dB | ' + (f'medido {m:.2f} dB | penalizacion {m-t:.2f} dB' if m else 'medido: no alcanza ese nivel'))
with open(ruta.with_name('resumen_por_EbN0.csv'), 'w', newline='') as f:
    w = csv.writer(f); w.writerow(tab[0].keys())
    for r in tab: w.writerow(r.values())
try:
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
    for a, (nom, f, cp, cm) in zip(ax[:2], (('BER', ber_t, 'bp', 'bm'), ('SER', ser_t, 'sp', 'sm'))):
        a.semilogy(xx, [f(e) for e in xx], 'k-', label='Teoria 16-QAM')
        a.semilogy(x, np.maximum([r[cp] for r in tab], 1e-7), 'o-', color='#5f860f', label='Medido (promedio)')
        a.semilogy(x, np.maximum([r[cm] for r in tab], 1e-7), 's--', color='#b03a2e', label='Medido (mediana)')
        a.set_ylim(1e-6, 1); a.set_xlabel('Eb/N0 aplicado [dB]'); a.set_ylabel(nom); a.grid(True, which='both', alpha=.3); a.legend()
    ax[2].plot(x, x, 'k-', label='SNR = Eb/N0'); ax[2].plot(x, [r['snr'] for r in tab], 'o-', color='#5f860f', label='measured_snr_db')
    a2 = ax[2].twinx(); a2.plot(x, [r['disp'] for r in tab], 's--', color='#b03a2e'); a2.set_ylabel('iq_dispersion_rms', color='#b03a2e')
    ax[2].set_xlabel('Eb/N0 aplicado [dB]'); ax[2].set_ylabel('SNR medida [dB]'); ax[2].grid(alpha=.3); ax[2].legend()
    plt.tight_layout(); plt.savefig(ruta.with_name('analisis_BER_SER.png'), dpi=140); print('\nGuardado analisis_BER_SER.png y resumen_por_EbN0.csv')
except ImportError:
    print('\n(matplotlib no esta instalado: no se genero la grafica; resumen_por_EbN0.csv guardado)')
