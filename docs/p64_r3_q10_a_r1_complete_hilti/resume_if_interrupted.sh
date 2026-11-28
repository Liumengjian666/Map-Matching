#!/usr/bin/env bash
set -euo pipefail
d=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$d/../.."
which_asset="$1"
case "$which_asset" in
  map) filename=sheldonian_full.e57; workers=20 ;;
  bag) filename=exp18_corridor_lower_gallery_2.bag; workers=32 ;;
  *) exit 2 ;;
esac
pidfile=run/r1/$which_asset.pid
log=run/r1/${which_asset}_download.log
for attempt in {1..3}; do
  if [[ -s "$pidfile" ]]; then
    read -r running <"$pidfile"
    while kill -0 "$running" 2>/dev/null && grep -aq "fetch_ranges.py" "/proc/$running/cmdline"; do
      sleep 15
    done
  fi
  if [[ -f run/assets/"$filename" ]]; then
    exit 0
  fi
  # Full SHA mismatch is a received-data integrity failure, not a retryable transport timeout.
  if grep -q 'official SHA mismatch' "$log"; then
    printf '%s %s\n' "$which_asset" 'checksum failed; preserve evidence' >&2
    exit 3
  fi
  printf '%s %s resume attempt %d\n' "$(date -Iseconds)" "$which_asset" "$attempt" >>"$log"
  python3 docs/p64_r3_q10_public_mechanism_validation/fetch_ranges.py run/assets "$which_asset" --workers "$workers" >>"$log" 2>&1 &
  running=$!
  printf '%s\n' "$running" >"$pidfile"
  if wait "$running"; then
    exit 0
  fi
done
printf '%s %s\n' "$which_asset" 'automatic retries exhausted' >&2
exit 1
