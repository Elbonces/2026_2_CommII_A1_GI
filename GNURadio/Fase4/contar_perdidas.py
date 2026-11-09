"""Cuenta ventanas 'perdidas' (SER > 0.5, error casi total por perdida de sincronismo) y promedia solo las enganchadas.
Uso: python3 contar_perdidas.py metricas_sistema_GrupoA1-GI.csv"""
import sys, csv
from collections import defaultdict
filas = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else 'metricas_sistema_GrupoA1-GI.csv')))
d = defaultdict(lambda: [0, 0, 0.0, 0.0, 0])
for f in filas:
    e = float(f['ebn0_db']); s = float(f['measured_ser']); b = float(f['measured_ber']); v = d[e]
    v[0] += 1
    if s > 0.5: v[1] += 1
    else: v[2] += s; v[3] += b; v[4] += 1
print(f"{'Eb/N0':>6} {'ventanas':>9} {'perdidas':>9} {'%perd':>6} {'SER_enganchado':>15} {'BER_enganchado':>15}")
for e in sorted(d):
    n, l, s, b, k = d[e]
    print(f"{e:6.1f} {n:9d} {l:9d} {100*l/n:6.1f} {(s/k if k else 0):15.3e} {(b/k if k else 0):15.3e}")
