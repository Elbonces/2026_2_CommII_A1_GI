import json, math, numpy as np, sys
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.special import erfc
from scipy.signal import welch
OUT='/home/claude/fase2/figs/'
C1,C2,C3='#2a78d6','#eb6834','#1baf7a'; TXT='#0b0b0b'; TX2='#52514e'; GRID='#e4e3df'
plt.rcParams.update({'font.size':9,'axes.edgecolor':TX2,'axes.labelcolor':TXT,'xtick.color':TX2,'ytick.color':TX2,
 'axes.grid':True,'grid.color':GRID,'grid.linewidth':0.6,'axes.spines.top':False,'axes.spines.right':False,'font.family':'DejaVu Sans','lines.linewidth':2})
pts=np.array([-3-3j,-3-1j,-3+3j,-3+1j,-1-3j,-1-1j,-1+3j,-1+1j,3-3j,3-1j,3+3j,3+1j,1-3j,1-1j,1+3j,1+1j])/math.sqrt(10)
# 1) Constelacion Gray
fig,ax=plt.subplots(figsize=(3.4,3.1))
for v in (-2/math.sqrt(10),0,2/math.sqrt(10)):
    ax.axvline(v,color=TX2,ls='--',lw=0.8); ax.axhline(v,color=TX2,ls='--',lw=0.8)
ax.scatter(pts.real,pts.imag,s=46,color=C1,zorder=3,edgecolor='white',linewidth=1.2)
for i,p in enumerate(pts):
    ax.annotate(format(i,'04b'),(p.real,p.imag),textcoords='offset points',xytext=(0,7),ha='center',fontsize=7.5,color=TXT)
ax.set_xlim(-1.35,1.35); ax.set_ylim(-1.35,1.35); ax.set_aspect('equal')
ax.set_xlabel('I'); ax.set_ylabel('Q')
ax.set_xticks([-3/math.sqrt(10),-1/math.sqrt(10),1/math.sqrt(10),3/math.sqrt(10)]); ax.set_xticklabels(['-0,95','-0,32','0,32','0,95'])
ax.set_yticks([-3/math.sqrt(10),-1/math.sqrt(10),1/math.sqrt(10),3/math.sqrt(10)]); ax.set_yticklabels(['-0,95','-0,32','0,32','0,95'])
ax.grid(False)
fig.tight_layout(); fig.savefig(OUT+'constelacion_gray.pdf'); plt.close(fig)

# 2) PSD para distintos beta, generada con los mismos bloques de GNU Radio
from gnuradio import gr, blocks, filter as grf
from gnuradio.filter import firdes
Rs=3e6; Sps=8; fs=Rs*Sps; rng=np.random.default_rng(1)
sym=pts[rng.integers(0,16,200000)].astype(np.complex64)
fig,ax=plt.subplots(figsize=(3.45,2.6))
for b,c,lab in [(0.20,C3,'β = 0,20'),(0.35,C1,'β = 0,35 (diseño)'),(0.50,C2,'β = 0,50')]:
    tb=gr.top_block(); src=blocks.vector_source_c(sym.tolist(),False)
    f=grf.interp_fir_filter_ccf(Sps, firdes.root_raised_cosine(Sps,fs,Rs,b,11*Sps)); snk=blocks.vector_sink_c()
    tb.connect(src,f,snk); tb.run(); x=np.array(snk.data())
    fr,P=welch(x,fs=fs,nperseg=4096,return_onesided=False,scaling='density',detrend=False); o=np.argsort(fr)
    P=10*np.log10(P[o]/P[o].max()); ax.plot(fr[o]/1e6,P,color=c,lw=1.6 if b!=0.35 else 2.2,label=lab)
ax.axvspan(-2.5,2.5,color='#d7e6f8',alpha=0.45,lw=0,label='Canal 5 MHz (Fase 1)')
ax.set_xlim(-4.5,4.5); ax.set_ylim(-80,5); ax.set_xlabel('Frecuencia [MHz]'); ax.set_ylabel('PSD normalizada [dB]')
ax.legend(fontsize=7,loc='lower center',frameon=False,ncol=2,bbox_to_anchor=(0.5,1.0),borderaxespad=0.2)
fig.tight_layout(); fig.savefig(OUT+'psd_beta.pdf'); plt.close(fig)

# 3) Curva BER medida vs teoria
d=json.load(open(sys.argv[1]))
pb=d['barrido_ber']
E=np.linspace(0,14,200); th=0.375*erfc(np.sqrt(0.4*10**(E/10)))
fig,ax=plt.subplots(figsize=(3.45,2.6))
ax.semilogy(E,th,color=TX2,lw=1.6,label='Teórica 16-QAM Gray')
for agc,c,mk,ms,lab in [(1,C1,'o',8,'GNU Radio, con AGC'),(0,C2,'x',6,'GNU Radio, sin AGC (ganancia ideal)')]:
    xs=[r['EbN0_dB'] for r in pb if r['agc']==agc and r['ber']>0]; ys=[r['ber'] for r in pb if r['agc']==agc and r['ber']>0]
    ax.semilogy(xs,ys,mk,color=c,ms=ms,mew=1.8,label=lab,lw=0,mec='white' if mk=='o' else c)
# puntos de operacion
for lab,eb,yy in [('LOS',30-6.02,None)]:
    pass
ax.axhline(1e-5,color=TX2,ls=':',lw=1); ax.text(13.9,1.4e-5,'objetivo $10^{-5}$',fontsize=7,color=TX2,ha='right')
ax.set_xlabel('$E_b/N_0$ [dB]'); ax.set_ylabel('BER'); ax.set_ylim(1e-6,0.5); ax.set_xlim(0,14)
ax.legend(fontsize=7,frameon=False,loc='lower left')
fig.tight_layout(); fig.savefig(OUT+'ber_curva.pdf'); plt.close(fig)
print('ok')
