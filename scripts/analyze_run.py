"""Generate traceable plots and metrics without substituting commands for measurements."""
import csv
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder = Path(sys.argv[1])
rows = list(csv.DictReader((folder/'execution.csv').open()))
def col(name):
    return np.array([float(r[name]) if r.get(name) not in ('', None) else np.nan for r in rows])
t = col('t_s')
source = {r['source'] for r in rows}
assert len(source) == 1, 'Do not mix simulation and hardware evidence'
source = source.pop()
out = folder/'analysis'
out.mkdir(exist_ok=True)
plt.rcParams.update({'font.size':11, 'axes.spines.top':False, 'axes.spines.right':False})
fig, ax = plt.subplots(2,1,figsize=(9,6),sharex=True,constrained_layout=True)
for a, keys, labels, unit in [
    (ax[0],['x_ref_m','range_progress_m'],['Cubic x reference','Range-derived progress'],'Position (m)'),
    (ax[1],['v_ref_mps','vx_measured_mps','vx_mps'],['Cubic speed reference','Measured body speed','Command'],'Speed (m/s)')]:
    for k,l in zip(keys,labels): a.plot(t,col(k),label=l)
    a.set_ylabel(unit); a.grid(alpha=.2); a.legend(fontsize=9)
ax[0].set_title(source+' - reference and response')
ax[1].set_xlabel('Elapsed time (s)')
fig.savefig(out/'references.png',dpi=180); plt.close(fig)
fig, ax = plt.subplots(2,1,figsize=(9,6),sharex=True,constrained_layout=True)
for a,k,label in zip(ax,['range_tracking_error_m','velocity_tracking_error_mps'],['Range progress error (m)','Speed error (m/s)']):
    a.plot(t,col(k),color='#b64c2a'); a.axhline(0,color='gray',lw=.7); a.set_ylabel(label); a.grid(alpha=.2)
ax[0].set_title(source+' - reference minus response')
ax[1].set_xlabel('Elapsed time (s)')
fig.savefig(out/'tracking_errors.png',dpi=180); plt.close(fig)
fig, ax = plt.subplots(2,1,figsize=(9,6),sharex=True,constrained_layout=True)
ax[0].plot(t,col('vx_mps')); ax[0].set_ylabel('Forward command (m/s)')
ax[1].plot(t,col('V1_V'),color='#b64c2a'); ax[1].set_ylabel('Model V1 (V)')
ax[0].set_title(source+' - command and voltage estimate')
for a in ax: a.grid(alpha=.2)
ax[1].set_xlabel('Elapsed time (s)')
fig.savefig(out/'speed_voltage.png',dpi=180); plt.close(fig)
chasing = np.array([r['state']=='CHASING' for r in rows])
metrics = {'source':source,'samples':len(rows),'tracking_position_basis':'initial tag range minus current tag range; not independent Cartesian odometry'}
for name in ['range_tracking_error_m','velocity_tracking_error_mps']:
    values=col(name)[chasing]; values=values[np.isfinite(values)]
    metrics[name]={'samples':len(values),'rmse':float(np.sqrt(np.mean(values**2))) if len(values) else None,'max_abs':float(np.max(np.abs(values))) if len(values) else None}
(out/'metrics.json').write_text(json.dumps(metrics,indent=2))
# Compact presentation data retains exact recorded values, sampled for readability.
idx=np.arange(0,len(rows),5)
deck={'source':source,'t':t[idx].tolist()}
for k in ['x_ref_m','range_progress_m','v_ref_mps','vx_measured_mps','range_tracking_error_m','velocity_tracking_error_mps']:
    deck[k]=[float(v) if np.isfinite(v) else None for v in col(k)[idx]]
(out/'chart_data.json').write_text(json.dumps(deck))
print(json.dumps(metrics,indent=2))
