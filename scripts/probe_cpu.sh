#!/usr/bin/env bash
# B9: which incoming messages cost the bridge its CPU? Starts W2 slam (drone on the ground), measures the bridge,
# then four tiny probe nodes one after another (15 s each), stops.  About 3 minutes.
set -uo pipefail
REPO=$HOME/drone_ws/src
OUT=$HOME/drone_results/$(date +%Y%m%d_%H%M%S)_cpu_probe
mkdir -p "$OUT"
"$REPO/scripts/sim_up.sh" w2_turns_junctions slam "$OUT"
echo "waiting 60 s for SLAM and the bridge to settle ..."; sleep 60
PID=$(pgrep -f 'lib/drone_bridge/bridge_node' | head -1)
{
python3 - "$PID" <<'PY'
import sys, time, psutil
p = psutil.Process(int(sys.argv[1])); p.cpu_percent(None); time.sleep(10)
print(f"bridge       : CPU {p.cpu_percent(None):5.1f}% of one core")
PY
for m in none clock tf both; do
  bash -c "source /opt/ros/humble/setup.bash; source $HOME/drone_ws/install/setup.bash; python3 $REPO/drone_sim/tools/cpu_probe.py --mode $m" 2>/dev/null
done
} | tee "$OUT/probe.txt"
"$REPO/scripts/sim_down.sh"
echo "saved: $OUT/probe.txt"
