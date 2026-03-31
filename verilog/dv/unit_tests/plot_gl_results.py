#!/usr/bin/env python3
"""Generate waveform-style visualization from GL simulation results."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

fig, axes = plt.subplots(5, 1, figsize=(14, 10), sharex=True,
                         gridspec_kw={'height_ratios': [1, 2, 2, 1, 1]})
fig.suptitle('SenseEdge ASIC — Gate-Level Simulation Results\n(SKY130, 20 MHz, 44,409 gates)',
             fontsize=14, fontweight='bold', y=0.98)

# Time axis (simplified pipeline stages)
stages = ['Reset', 'Weight\nLoad', 'SPI\nAcquire', 'FFT\nCompute', 'Feature\nExtract', 'NN\nInference', 'Alarm\nCheck', 'Idle']
x = np.arange(len(stages))

# 1. Clock (20 MHz)
ax = axes[0]
clk = np.array([0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1])
t_clk = np.linspace(0, 7, len(clk))
ax.step(t_clk, clk, 'b-', linewidth=0.8, where='post')
ax.set_ylabel('wb_clk_i', fontsize=9)
ax.set_ylim(-0.2, 1.4)
ax.set_yticks([0, 1])
ax.set_yticklabels(['0', '1'], fontsize=8)

# 2. FFT Magnitude Bins (from GL sim output)
ax = axes[1]
bins = np.zeros(16)
bins[8] = 32020  # From GL simulation output
ax.bar(range(16), bins, color='#2196F3', width=0.7)
ax.set_ylabel('FFT Mag', fontsize=9)
ax.set_title('FFT Output — GL matches RTL: Bin[8] = 32,020', fontsize=9, loc='left', color='green')
ax.set_xticks(range(16))
ax.set_xticklabels([str(i) for i in range(16)], fontsize=7)
ax.set_xlabel('Frequency Bin', fontsize=8)

# 3. Pipeline Status (timing diagram style)
ax = axes[2]
signals = {
    'enable':    [0, 0, 1, 1, 1, 1, 1, 0],
    'fft_busy':  [0, 0, 0, 1, 0, 0, 0, 0],
    'fe_busy':   [0, 0, 0, 0, 1, 0, 0, 0],
    'nn_busy':   [0, 0, 0, 0, 0, 1, 0, 0],
}
colors = ['#4CAF50', '#2196F3', '#FF9800', '#E91E63']
for idx, (name, vals) in enumerate(signals.items()):
    offset = idx * 1.5
    ax.step(x, [v + offset for v in vals], color=colors[idx], linewidth=2, where='post')
    ax.fill_between(x, offset, [v + offset for v in vals], alpha=0.3, step='post', color=colors[idx])
    ax.text(-0.7, offset + 0.5, name, fontsize=8, va='center', fontweight='bold', color=colors[idx])
ax.set_ylim(-0.5, 6.5)
ax.set_yticks([])
ax.set_ylabel('Pipeline', fontsize=9)

# 4. GPIO Outputs
ax = axes[3]
gpio_alarm =  [0, 0, 0, 0, 0, 0, 0, 0]
gpio_led =    [0, 0, 1, 1, 1, 1, 1, 0]
ax.step(x, gpio_led, 'g-', linewidth=2, where='post', label='LED (GPIO[4])')
ax.step(x, [v + 1.5 for v in gpio_alarm], 'r-', linewidth=2, where='post', label='Alarm (GPIO[3])')
ax.set_ylabel('GPIO', fontsize=9)
ax.set_ylim(-0.3, 3)
ax.set_yticks([])
ax.legend(loc='right', fontsize=7)

# 5. Classification Result
ax = axes[4]
class_names = ['—', '—', '—', '—', '—', '—', 'Class 0\n(Healthy)', '—']
class_colors = ['white', 'white', 'white', 'white', 'white', 'white', '#C8E6C9', 'white']
for i, (name, color) in enumerate(zip(class_names, class_colors)):
    ax.add_patch(mpatches.FancyBboxPatch((i-0.4, 0.1), 0.8, 0.8,
                 boxstyle="round,pad=0.1", facecolor=color, edgecolor='gray'))
    if name != '—':
        ax.text(i, 0.5, name, ha='center', va='center', fontsize=8, fontweight='bold')
ax.set_xlim(-0.5, 7.5)
ax.set_ylim(0, 1)
ax.set_yticks([])
ax.set_ylabel('Result', fontsize=9)
ax.set_xticks(x)
ax.set_xticklabels(stages, fontsize=8)

# Results box
results_text = (
    'GL Simulation Results (Icarus Verilog 12.0)\n'
    '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
    'Weight Loading:  PASS (212 params)\n'
    'FFT Output:        PASS (Bin[8]=32020)\n'
    'GPIO Directions: PASS\n'
    'System Control:  PASS\n'
    '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
    'Gate Count: 44,409 | Process: SKY130'
)
fig.text(0.78, 0.55, results_text, fontsize=8, fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='#E8F5E9', edgecolor='green', alpha=0.9),
         verticalalignment='center')

plt.tight_layout(rect=[0, 0, 0.75, 0.95])
plt.savefig('/Users/fidelmakatia/chipfoundry/senseedge/docs/gl_simulation_results.png',
            dpi=150, bbox_inches='tight', facecolor='white')
print("Saved: docs/gl_simulation_results.png")
