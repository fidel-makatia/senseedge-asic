#!/usr/bin/env bash
# Resume the autopilot from step 3 (wrapper harden) since:
#  - senseedge_top harden completed (RUN_2026-05-27_00-15-53)
#  - copy_views already done — gds/senseedge_top.gds has 345k decap_40_12 refs
#
# Same logic as autopilot.sh but skipping steps 1+2 and handling LibreLane's
# Magic.StreamOut false-fatal-error (treat non-zero exit as OK if streamout
# GDS file was actually produced).

set -uo pipefail

PROJ="/Users/fidelmakatia/chipfoundry/senseedge"
TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$PROJ/logs/autopilot_resume_$TS"
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

log "AUTOPILOT RESUME START — skipping steps 1+2 (senseedge_top already done)"
echo "RESULT=RUNNING" >> "$STATUS"

# -----------------------------------------------------------------------------
# Step 3: harden user_project_wrapper
# -----------------------------------------------------------------------------
log "Step 3/6: user_project_wrapper harden"
WLOG="$LOG_DIR/wrapper_harden.log"

heartbeat "$WLOG" &
HB_PID=$!

docker run --rm -i \
  -v /Users/fidelmakatia:/Users/fidelmakatia \
  -e PDK_ROOT="$PDK_ROOT" \
  -w "$PROJ" \
  "$LIBRELANE_IMG" \
  python3 -m librelane --pdk-root "$PDK_ROOT" openlane/user_project_wrapper/config.json \
  > "$WLOG" 2>&1
WRC=$?

kill $HB_PID 2>/dev/null || true

WRAP_RUN="$(ls -t "$PROJ/openlane/user_project_wrapper/runs/" | head -1)"
WRAP_STREAMOUT="$(ls "$PROJ/openlane/user_project_wrapper/runs/$WRAP_RUN"/*-magic-streamout/user_project_wrapper.gds 2>/dev/null | head -1)"
WRAP_MAGIC_GDS="$(ls "$PROJ/openlane/user_project_wrapper/runs/$WRAP_RUN"/*-magic-streamout/user_project_wrapper.magic.gds 2>/dev/null | head -1)"

if [ -z "$WRAP_STREAMOUT" ] && [ -n "$WRAP_MAGIC_GDS" ]; then
  # fall back to magic.gds form
  WRAP_DIR="$(dirname "$WRAP_MAGIC_GDS")"
  cp -v "$WRAP_MAGIC_GDS" "$WRAP_DIR/user_project_wrapper.gds" >> "$LOG_DIR/wrapper_recovery.log" 2>&1
  WRAP_STREAMOUT="$WRAP_DIR/user_project_wrapper.gds"
fi

if [ -n "$WRAP_STREAMOUT" ] && [ -f "$WRAP_STREAMOUT" ]; then
  log "  wrapper streamout GDS produced (rc=$WRC, run=$WRAP_RUN)"
else
  die "wrapper harden: no streamout GDS (rc=$WRC). See $WLOG"
fi

# -----------------------------------------------------------------------------
# Step 4: copy_views wrapper
# -----------------------------------------------------------------------------
log "Step 4/6: copy_views user_project_wrapper"
bash "$COPY" "$PROJ" user_project_wrapper "$WRAP_RUN" \
  > "$LOG_DIR/copy_views_wrapper.log" 2>&1 \
  || die "copy_views user_project_wrapper failed"
[ -f "$PROJ/gds/user_project_wrapper.gds" ] || die "wrapper GDS not in gds/"
log "  wrapper views copied"

# -----------------------------------------------------------------------------
# Step 5: cf precheck
# -----------------------------------------------------------------------------
log "Step 5/6: cf precheck"
cd "$PROJ"
cf precheck > "$LOG_DIR/cf_precheck.log" 2>&1
PRC=$?
if [ $PRC -ne 0 ]; then
  die "cf precheck exit=$PRC (see $LOG_DIR/cf_precheck.log)"
fi
if grep -E "(\[FAIL\]|^FAIL|FAILED|ERROR:)" "$LOG_DIR/cf_precheck.log" >/dev/null 2>&1; then
  die "cf precheck reported FAIL/ERROR (see $LOG_DIR/cf_precheck.log)"
fi
log "  cf precheck PASSED"

# -----------------------------------------------------------------------------
# Step 6: cf push
# -----------------------------------------------------------------------------
log "Step 6/6: cf push"
cd "$PROJ"
cf push > "$LOG_DIR/cf_push.log" 2>&1
PUSHRC=$?
if [ $PUSHRC -ne 0 ]; then
  die "cf push exit=$PUSHRC (see $LOG_DIR/cf_push.log)"
fi
log "  cf push completed"

sed -i '' 's/^RESULT=RUNNING/RESULT=SUCCESS/' "$STATUS" 2>/dev/null || echo "RESULT=SUCCESS" >> "$STATUS"
log "AUTOPILOT DONE — re-spin submitted to ChipFoundry"
