#!/usr/bin/env bash
# Run a batch of test commands unattended in a tmux session "batch".
# Usage:   scripts/batch.sh NAME 'command1; command2; ...'
# Watch:   tmux attach -t batch        (leave again: Ctrl+b then d)
# Status:  scripts/batch.sh status
# Done when the log's last line is "BATCH DONE ..." and `tmux ls` no longer lists "batch".
set -euo pipefail
REPO=$HOME/drone_ws/src
if [ "${1:-}" = status ]; then
  LOG=$(ls -t $HOME/drone_results/batch_*.log 2>/dev/null | head -1)
  [ -z "$LOG" ] && { echo "no batch log yet"; exit 0; }
  echo "log: $LOG"
  if tmux has-session -t batch 2>/dev/null; then echo "state: RUNNING"; else echo "state: FINISHED (or stopped)"; fi
  echo "runs started: $(grep -c '^up: tmux attach' "$LOG")   PASS: $(grep -cE '^A[0-9.]+: PASS' "$LOG")   FAIL: $(grep -cE '^A[0-9.]+: FAIL' "$LOG")"
  D=$(grep -c '^done:' "$LOG"); [ "$D" -gt 0 ] && echo "replays done: $D   PASS: $(grep -c '^done:.*PASS' "$LOG")   FAIL: $(grep -c '^done:.*FAIL' "$LOG")"
  echo "--- last lines:"; tail -4 "$LOG"
  exit 0
fi
NAME=${1:?name}; CMDS=${2:?commands}
if tmux has-session -t batch 2>/dev/null; then echo "a batch is already running (scripts/batch.sh status)"; exit 1; fi
LOG=$HOME/drone_results/batch_$(date +%Y%m%d_%H%M%S)_$NAME.log
mkdir -p "$HOME/drone_results"
tmux new-session -d -s batch "bash -c 'cd $REPO; { $CMDS; } 2>&1 | tee $LOG; echo \"BATCH DONE \$(date +%H:%M)\" | tee -a $LOG; sleep 5'"
echo "batch '$NAME' started. Log: $LOG"
echo "check: scripts/batch.sh status      watch: tmux attach -t batch (Ctrl+b d to leave)"
