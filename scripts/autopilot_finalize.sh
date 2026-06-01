#!/usr/bin/env bash
# Final autopilot pass — vendor GDS now contains decap_40_12 baked in.
# Step A: re-stream senseedge_top (just Magic.StreamOut, ~2 min)
# Step B: re-harden user_project_wrapper from scratch (~5 min)
# Step C: cf precheck
# Step D: cf push

set -uo pipefail

PROJ="/Users/fidelmakatia/chipfoundry/senseedge"
TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$PROJ/logs/autopilot_finalize_$TS"
mkdir -p "$LOG_DIR"
STATUS="$LOG_DIR/STATUS.txt"
LIBRELANE_IMG="ghcr.io/librelane/librelane:2.4.6"
PDK_ROOT="$PROJ/dependencies/pdks"
COPY="$PROJ/scripts/copy_views_streamout.sh"

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" >> "$STATUS"; }
die() { log "FAIL: $*"; echo "RESULT=FAIL" >> "$STATUS"; exit 1; }

heartbeat() {
  local lname="$1"
  while sleep 60; do
    [ -f "$lname" ] || continue
    local stage
    stage="$(grep -E "^\[.*\] .*Running '" "$lname" 2>/dev/null | tail -1 | sed -E "s/.*Running '([^']+)'.*/\\1/")"
    [ -n "$stage" ] && echo "[$(date '+%H:%M:%S')] heartbeat: $stage" >> "$STATUS"
  done
}

log "AUTOPILOT FINALIZE START"
echo "RESULT=RUNNING" >> "$STATUS"

# -----------------------------------------------------------------------------
# Step A: re-stream senseedge_top using the existing run + new vendor GDS
# -----------------------------------------------------------------------------
TOP_RUN="RUN_2026-05-27_00-15-53"
log "Step A: re-stream senseedge_top ($TOP_RUN, only Magic.StreamOut)"
ALOG="$LOG_DIR/restream_top.log"

docker run --rm -i \
  -v /Users/fidelmakatia:/Users/fidelmakatia \
  -e PDK_ROOT="$PDK_ROOT" \
  -w "$PROJ" \
  "$LIBRELANE_IMG" \
  python3 -m librelane --pdk-root "$PDK_ROOT" \
    --only Magic.StreamOut --last-run \
    openlane/senseedge_top/config.json \
  > "$ALOG" 2>&1
ARC=$?

NEW_GDS="$(ls "$PROJ/openlane/senseedge_top/runs/$TOP_RUN"/*-magic-streamout/senseedge_top.gds 2>/dev/null | head -1)"
NEW_MGDS="$(ls "$PROJ/openlane/senseedge_top/runs/$TOP_RUN"/*-magic-streamout/senseedge_top.magic.gds 2>/dev/null | head -1)"
if [ -z "$NEW_GDS" ] && [ -n "$NEW_MGDS" ]; then
  cp -v "$NEW_MGDS" "$(dirname "$NEW_MGDS")/senseedge_top.gds" >> "$ALOG" 2>&1
  NEW_GDS="$(dirname "$NEW_MGDS")/senseedge_top.gds"
fi
[ -f "$NEW_GDS" ] || die "re-stream produced no GDS (rc=$ARC, see $ALOG)"

# Sanity: confirm decap_40_12 references AND no BM_ prefix corruption
DECAP_N=$(strings "$NEW_GDS" 2>/dev/null | grep -c "decap_40_12")
BM_N=$(strings "$NEW_GDS" 2>/dev/null | grep -c "^BM_")
log "  re-streamed senseedge_top: decap_40_12=$DECAP_N, BM_*=$BM_N"
[ "$DECAP_N" -gt 100 ] || die "expected many decap_40_12 references, got only $DECAP_N"
[ "$BM_N" -eq 0 ] || die "GDS still has BM_-prefix corruption ($BM_N refs)"

bash "$COPY" "$PROJ" senseedge_top "$TOP_RUN" \
  > "$LOG_DIR/copy_views_top.log" 2>&1 \
  || die "copy_views senseedge_top failed"
log "  senseedge_top views copied"

# -----------------------------------------------------------------------------
# Step B: re-harden user_project_wrapper from scratch
# -----------------------------------------------------------------------------
WLOG="$LOG_DIR/wrapper_harden.log"
log "Step B: re-harden user_project_wrapper"

heartbeat "$WLOG" &
HB_PID=$!

docker run --rm -i \
  -v /Users/fidelmakatia:/Users/fidelmakatia \
  -e PDK_ROOT="$PDK_ROOT" \
  -w "$PROJ" \
  "$LIBRELANE_IMG" \
  python3 -m librelane --pdk-root "$PDK_ROOT" \
    openlane/user_project_wrapper/config.json \
  > "$WLOG" 2>&1
WRC=$?
kill $HB_PID 2>/dev/null || true

WRAP_RUN="$(ls -t "$PROJ/openlane/user_project_wrapper/runs/" | head -1)"
WRAP_GDS="$(ls "$PROJ/openlane/user_project_wrapper/runs/$WRAP_RUN"/*-magic-streamout/user_project_wrapper.gds 2>/dev/null | head -1)"
WRAP_MGDS="$(ls "$PROJ/openlane/user_project_wrapper/runs/$WRAP_RUN"/*-magic-streamout/user_project_wrapper.magic.gds 2>/dev/null | head -1)"
if [ -z "$WRAP_GDS" ] && [ -n "$WRAP_MGDS" ]; then
  cp -v "$WRAP_MGDS" "$(dirname "$WRAP_MGDS")/user_project_wrapper.gds" >> "$WLOG" 2>&1
  WRAP_GDS="$(dirname "$WRAP_MGDS")/user_project_wrapper.gds"
fi
[ -f "$WRAP_GDS" ] || die "wrapper streamout produced no GDS (rc=$WRC, see $WLOG)"
log "  wrapper streamout GDS: $WRAP_GDS (run=$WRAP_RUN, rc=$WRC)"

bash "$COPY" "$PROJ" user_project_wrapper "$WRAP_RUN" \
  > "$LOG_DIR/copy_views_wrap.log" 2>&1 \
  || die "copy_views wrapper failed"

# Sanity: wrapper GDS should reference senseedge_top and have no BM_-prefix corruption
BM_W=$(strings "$PROJ/gds/user_project_wrapper.gds" 2>/dev/null | grep -c "^BM_")
[ "$BM_W" -eq 0 ] || die "wrapper GDS has BM_-prefix corruption ($BM_W)"
log "  wrapper views copied; no BM_ corruption"

# -----------------------------------------------------------------------------
# Step C: cf precheck
# -----------------------------------------------------------------------------
log "Step C: cf precheck"
cd "$PROJ"
cf precheck > "$LOG_DIR/cf_precheck.log" 2>&1
PRC=$?
if [ $PRC -ne 0 ]; then
  die "cf precheck exit=$PRC (see $LOG_DIR/cf_precheck.log)"
fi
if grep -E "(\[FAIL\]|^FAIL|FAILED|ERROR:)" "$LOG_DIR/cf_precheck.log" >/dev/null 2>&1; then
  die "cf precheck FAIL/ERROR lines present (see $LOG_DIR/cf_precheck.log)"
fi
log "  cf precheck PASSED"

# -----------------------------------------------------------------------------
# Step D: cf push
# -----------------------------------------------------------------------------
log "Step D: cf push"
cd "$PROJ"
cf push > "$LOG_DIR/cf_push.log" 2>&1
PUSHRC=$?
if [ $PUSHRC -ne 0 ]; then
  die "cf push exit=$PUSHRC (see $LOG_DIR/cf_push.log)"
fi
log "  cf push completed"

sed -i '' 's/^RESULT=RUNNING/RESULT=SUCCESS/' "$STATUS" 2>/dev/null || echo "RESULT=SUCCESS" >> "$STATUS"
log "AUTOPILOT FINALIZE DONE — re-spin submitted to ChipFoundry"
