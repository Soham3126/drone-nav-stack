import json, os, yaml, sys
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
root = sys.argv[1]; out = sys.argv[2]
names = sorted(os.listdir(os.path.join(root, 'maps')))
def draw(ax, n, small=False):
    m = json.load(open(os.path.join(root, 'maps', n, 'meta.json')))
    for r in m['rooms']:
        ax.add_patch(Rectangle((r['x0'], r['y0']), r['x1']-r['x0'], r['y1']-r['y0'], color='#fde7c8', lw=0, zorder=0))
    p = os.path.join(root, 'patterns', n)
    if os.path.isdir(p):
        f = os.path.join(p, os.listdir(p)[0]); pt = yaml.safe_load(open(f))
        xs, ys = zip(*[(w[0], w[1]) for w in pt['waypoints']])
        ax.plot(xs, ys, color='#3b82f6', lw=0.8 if small else 1.2, alpha=0.6, zorder=1,
                label=f"flight path ({pt['pattern']}, {pt.get('path_length_m','?')} m)")
    for w in m['walls']:
        ax.add_patch(Rectangle((w[0], w[1]), w[2]-w[0], w[3]-w[1], color='#1f2937', lw=0, zorder=2))
    for b in m['boxes']:
        ax.add_patch(Rectangle((b['x']-b['sx']/2, b['y']-b['sy']/2), b['sx'], b['sy'], color='#dc2626', zorder=3, label='tall box'))
    for b in m['low_boxes']:
        ax.add_patch(Rectangle((b['x']-b['sx']/2, b['y']-b['sy']/2), b['sx'], b['sy'], color='#f59e0b', zorder=3, label='low box (under LiDAR)'))
    s = m['start']; ax.plot(s['x'], s['y'], marker='*', color='#16a34a', ms=10 if small else 14, zorder=4, ls='', label='start')
    ax.set_aspect('equal'); ax.autoscale_view(); ax.margins(0.03)
    ax.set_title(f"{n}\n{m['layout']}", fontsize=7 if small else 10)
    ax.tick_params(labelsize=6 if small else 8); ax.grid(alpha=0.15)
    if not small:
        ax.set_xlabel('east (m)'); ax.set_ylabel('north (m)')
        h, l = ax.get_legend_handles_labels(); d = dict(zip(l, h))
        ax.legend(d.values(), d.keys(), fontsize=7, loc='upper left', bbox_to_anchor=(1.01, 1))
for n in names:
    fig, ax = plt.subplots(figsize=(8, 6.5)); draw(ax, n)
    fig.tight_layout(); fig.savefig(os.path.join(out, f'{n}.png'), dpi=130); plt.close(fig)
cols = 4; rows = -(-len(names)//cols)
fig, axs = plt.subplots(rows, cols, figsize=(16, 4.3*rows))
for ax, n in zip(axs.flat, names): draw(ax, n, small=True)
for ax in list(axs.flat)[len(names):]: ax.axis('off')
fig.suptitle('All simulation worlds  (black = walls, orange = rooms, blue = test flight path, green star = start)', fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(out, 'all_worlds.png'), dpi=110); plt.close(fig)
