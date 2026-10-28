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
assert rows and all(r['source'] == 'SIMULATION' for r in rows)
states = {r['state'] for r in rows}
assert {'SEARCHING', 'ALIGNING', 'CHASING', 'ARRIVED'} <= states, states
assert max(float(r['vx_mps']) for r in rows) <= .16
assert all(float(r['vx_mps']) == 0 for r in rows if r['state'] in ('SEARCHING', 'ALIGNING', 'ARRIVED'))
arrived = [r for r in rows if r['state'] == 'ARRIVED']
assert float(arrived[0]['tag_distance_m']) <= .2
assert all(float(arrived[-1][f'V{i}_V']) == 0 for i in range(1, 5))
t = np.array([float(r['t_s']) for r in rows])
assert np.all(np.diff(t) > 0)
summary = dict(source='SIMULATION', samples=len(rows), states=sorted(states),
               stop_time_s=float(arrived[0]['t_s']), stop_range_m=float(arrived[0]['tag_distance_m']),
               max_speed_mps=max(float(r['vx_mps']) for r in rows))
(folder/'checks.json').write_text(json.dumps(summary, indent=2))
fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 5), constrained_layout=True)
for ax, col, label, color in zip(axes, ['vx_mps', 'V1_V'], ['Forward speed (m/s)', 'Model V1 (V)'], ['#176b99', '#be5a16']):
    ax.plot(t, [float(r[col]) for r in rows], color=color)
    ax.set_ylabel(label)
    ax.grid(alpha=.25)
axes[0].set_title('SIMULATION ONLY - ideal kinematic plant')
axes[1].set_xlabel('Elapsed time (s)')
fig.savefig(folder/'simulation_profile.png', dpi=180)
print(json.dumps(summary, indent=2))
