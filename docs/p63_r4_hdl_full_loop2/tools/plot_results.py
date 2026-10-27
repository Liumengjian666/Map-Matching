#!/usr/bin/env python3
"""Figures from every sealed estimate, including the stopped failure tail; no GT."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

A = Path(__file__).resolve().parents[1]
rows = list(csv.DictReader((A/'NDT_RESULTS.csv').open()))
result = json.loads((A/'results.json').read_text())
figures = A/'figures'
figures.mkdir(exist_ok=True)
data = lambda name: np.array([float(r[name]) for r in rows])
time = data('relative_s')
bad = np.array([r['external_valid'] != 'True' for r in rows])
fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), constrained_layout=True)
for axis, a, b in zip(axes, ('px', 'px'), ('py', 'pz')):
    x, y = data(a), data(b)
    axis.plot(x, y, linewidth=.75, color='tab:green', label='Native UKF estimate')
    axis.scatter(x[bad], y[bad], s=10, color='tab:red', label='External screen invalid', zorder=3)
    axis.scatter(x[0], y[0], marker='o', s=35, color='black', label='First output')
    axis.scatter(x[-1], y[-1], marker='x', s=55, color='purple', label='Stop output')
    axis.set(xlabel='Map X (m)', ylabel=f'Map {b[-1].upper()} (m)', aspect='equal')
    axis.grid(alpha=.2)
axes[0].legend(fontsize=8)
fig.suptitle('R4: all 3377 recorded estimates; no ground-truth accuracy claim')
fig.savefig(figures/'trajectory_estimate.png', dpi=180)
plt.close(fig)

fig, axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
axes[0].plot(time, data('inlier_fraction'), linewidth=.8)
axes[0].axhline(.50, color='tab:red', linestyle='--', label='Frozen external screen')
axes[0].set(ylabel='Native inlier fraction', ylim=(0, 1.05))
axes[0].legend(fontsize=8)
axes[1].plot(time, data('native_fitness_m2'), linewidth=.8, label='All-point NN fitness')
axes[1].plot(time, data('native_inlier_error_m2'), linewidth=.8, label='Origin-cropped inlier error')
axes[1].set(ylabel='Squared distance (m²)')
axes[1].legend(fontsize=8)
axes[2].plot(time, data('UKF_pred_to_NDT_translation_m'), linewidth=.8, label='Translation (m)')
axes[2].plot(time, data('UKF_pred_to_NDT_rotation_deg'), linewidth=.8, label='Rotation (deg)')
axes[2].set(ylabel='Prediction / NDT delta')
axes[2].legend(fontsize=8)
axes[3].plot(time, data('callback_ms'), linewidth=.6, label='Full callback with diagnostics')
axes[3].plot(time, data('ndt_compute_ms'), linewidth=.6, label='Native NDT compute')
axes[3].axhline(100, color='gray', linestyle='--', label='100 ms resource screen')
axes[3].set(ylabel='Time (ms)', xlabel='Seconds from first LiDAR header.stamp')
axes[3].legend(fontsize=8)
for axis in axes:
    axis.axvspan(time[-1], result['full_planned_sensor_span_s'], color='gray', alpha=.10)
    axis.axvspan(600, 650, color='tab:blue', alpha=.10)
    axis.axvline(time[-1], color='tab:red', linestyle=':', linewidth=.9)
    axis.set_xlim(0, result['full_planned_sensor_span_s'])
    axis.grid(alpha=.2)
axes[0].text(620, .65, '600–650 s: NOT REACHED', ha='center', fontsize=9)
axes[1].text(720, 1.5, 'Unprocessed after preregistered stop', ha='center', fontsize=9)
fig.suptitle('Frozen full-input scope; shaded region has no estimated samples')
fig.savefig(figures/'quality_runtime_and_unreached_scope.png', dpi=180)
plt.close(fig)
print('Saved two sealed-output figures; no localization or reference evaluation.')
