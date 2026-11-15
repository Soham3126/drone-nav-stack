#!/usr/bin/env bash
# collect_results.sh - copies the summary of every test's final run into results/ (for the repository).
#   scripts/collect_results.sh            (reads ~/drone_results, never changes it)
# Per test: the newest run made from committed code (not "-dirty") that passed; otherwise the newest such run.
# Only small summary files (report, result.json, meta, plots, cpu.csv) - no bags, no logs.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
RES=${RESULTS_DIR:-$HOME/drone_results}
TESTS="A0.1 A0.2 A0.3 A0.4 A1.1 A1.2 A1.3 A1.4 A1.5 A1.6 A1.7 A1.8 A1.9 A1.10 A1.11 B1 B2 B3 B4 B5 B6 B7 B8 B9 C0 C1 C2 C3 C4 C5 C6 C7"
OUT="$REPO/results"
rm -rf "$OUT"; mkdir -p "$OUT"
IDX="$OUT/INDEX.md"
{ echo "# Final run of every test"; echo
  echo "Collected by \`scripts/collect_results.sh\` on $(date '+%Y-%m-%d'). Summaries only (no recordings or logs)."
  echo "Each folder holds the run's \`report.txt\` (what was measured and the verdict), \`result.json\`, \`meta.txt\` (settings and git commit) and any plots."
  echo "Tests judged over many runs (B3-B7) are summarised in [docs/tasks](../docs/tasks)."; echo
  echo "| Test | Run | Result |"; echo "|---|---|---|"; } > "$IDX"
for id in $TESTS; do
  runs=$(ls -d "$RES"/*_"${id}"_* 2>/dev/null | grep -v -- '-dirty$' | sort -r || true)
  pick=""; verdict=""
  for d in $runs; do
    if grep -qs "PASS" "$d/report.txt" && ! grep -qs "${id}: FAIL" "$d/report.txt"; then pick=$d; verdict=PASS; break; fi
  done
  if [ -z "$pick" ] && [ -n "$runs" ]; then pick=$(echo "$runs" | head -1); verdict="see report (newest run; no passing run found)"; fi
  if [ -z "$pick" ]; then echo "| $id | - | no run found |" >> "$IDX"; continue; fi
  mkdir -p "$OUT/$id"
  find "$pick" -maxdepth 1 -type f -size -3M \( -name '*.txt' -o -name '*.json' -o -name '*.png' -o -name '*.csv' -o -name '*.yaml' \) \
       -exec cp {} "$OUT/$id/" \;
  echo "| $id | [$(basename "$pick")]($id/) | $verdict |" >> "$IDX"
done
du -sh "$OUT" | awk '{print "results/: " $1}'
echo "index: $IDX"
