#!/usr/bin/env bash
# Where does the bridge spend its CPU? (B9 found ~45% of one core; the book's limit is 10%.)
# Starts the slam profile in W2 (drone on the ground; Cartographer publishes TF at 200 Hz as in flight),
# samples the bridge for 30 s with py-spy (all threads, only the moments it is actually busy), stops.
# Needs once:  pip install py-spy      Usage:  scripts/profile_bridge.sh      (about 2 minutes, asks for sudo)
set -uo pipefail
REPO=$HOME/drone_ws/src
OUT=$HOME/drone_results/$(date +%Y%m%d_%H%M%S)_bridge_profile
mkdir -p "$OUT"
PYSPY=$(command -v py-spy || echo "$HOME/.local/bin/py-spy")
[ -x "$PYSPY" ] || { echo "py-spy not found: pip install py-spy"; exit 1; }
sudo -v || exit 1
"$REPO/scripts/sim_up.sh" w2_turns_junctions slam "$OUT"
echo "waiting 60 s for SLAM and the bridge to settle ..."; sleep 60
PID=$(pgrep -f 'lib/drone_bridge/bridge_node' | head -1)
if [ -z "$PID" ]; then echo "bridge process not found"; "$REPO/scripts/sim_down.sh"; exit 1; fi
echo "bridge pid $PID; CPU over 10 s (% of one core):"
python3 - "$PID" <<'PY' | tee "$OUT/cpu.txt"
import sys, time, psutil
p = psutil.Process(int(sys.argv[1])); p.cpu_percent(None); time.sleep(10)
print(f"total {p.cpu_percent(None):.1f}%  threads {p.num_threads()}")
PY
echo "sampling 30 s with py-spy ..."
sudo "$PYSPY" record --pid "$PID" --duration 30 --rate 200 --threads --format raw -o "$OUT/bridge_profile.txt" >/dev/null 2>&1
sudo chown "$USER" "$OUT/bridge_profile.txt" 2>/dev/null
"$REPO/scripts/sim_down.sh"
python3 - "$OUT/bridge_profile.txt" <<'PY' | tee "$OUT/summary.txt"
import sys, collections
tot, own, cum, thr = 0, collections.Counter(), collections.Counter(), collections.Counter()
for line in open(sys.argv[1]):
    stack, _, n = line.rstrip().rpartition(' ')
    n = int(n); tot += n
    frames = stack.split(';')
    thr[frames[0]] += n
    own[frames[-1]] += n
    for f in set(frames[1:]):
        cum[f] += n
print(f'{tot} busy samples')
print('--- by thread'); [print(f'{100*n/tot:5.1f}%  {k}') for k, n in thr.most_common(8)]
print('--- where the time is spent (own)'); [print(f'{100*n/tot:5.1f}%  {k}') for k, n in own.most_common(20)]
print('--- including what they call (cumulative)'); [print(f'{100*n/tot:5.1f}%  {k}') for k, n in cum.most_common(25)]
PY
echo "files: $OUT  (send me summary.txt)"
