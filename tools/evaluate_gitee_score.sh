#!/usr/bin/env bash
set -euo pipefail

# Run the evaluator shipped by the pinned Gitee SimEnv checkout.  The truth
# file is used only after the mission for local acceptance scoring; the
# exploration nodes never read it.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EVALUATOR="${GITEE_EVALUATOR:-$ROOT_DIR/src/SimEnv/src/building_obstacles/scripts/evaluate_danger.py}"
TRUTH_FILE="${TRUTH_FILE:-$ROOT_DIR/results/danger_truth.json}"
DETECTED_FILE="${DETECTED_FILE:-$ROOT_DIR/results/detected_danger.json}"
OUTPUT_FILE="${OUTPUT_FILE:-$ROOT_DIR/results/evaluation_result.json}"

for required in "$EVALUATOR" "$TRUTH_FILE" "$DETECTED_FILE"; do
  if [[ ! -f "$required" ]]; then
    echo "missing scoring input: $required" >&2
    exit 2
  fi
done

exec python3 "$EVALUATOR" \
  --truth-file "$TRUTH_FILE" \
  --detected-file "$DETECTED_FILE" \
  --output-file "$OUTPUT_FILE" \
  "$@"
