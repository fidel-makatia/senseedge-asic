#!/usr/bin/env bash
# Plan B: submit design with NO decap_12 and NO decap_40_12.
# Rationale: David's complaint was specifically about decap_12 causing poly
# density failure. With decap_12 absent (and decap_40_12 paths abandoned
# due to cross-PDK incompatibility), density may pass.
#
# Re-harden wrapper to integrate the May 26 14:17 senseedge_top.gds, then
# cf precheck, then cf push if precheck passes.

set -uo pipefail

PROJ="/Users/fidelmakatia/chipfoundry/senseedge"
TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$PROJ/logs/autopilot_planB_$TS"
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

log "AUTOPILOT PLAN B START — no large decap, integrating May 26 14:17 senseedge_top"
echo "RESULT=RUNNING" >> "$STATUS"

# Sanity: senseedge_top in place
[ "$(strings "$PROJ/gds/senseedge_top.gds" 2>/dev/null | grep -c "sky130_ef_sc_hd__decap_40_12")" -eq 0 ] \
  || die "senseedge_top.gds unexpectedly contains decap_40_12"
log "  senseedge_top.gds confirmed: no decap_40_12"

# -----------------------------------------------------------------------------
# Step 1: re-harden user_project_wrapper
# -----------------------------------------------------------------------------
WLOG="$LOG_DIR/wrapper_harden.log"
log "Step 1: re-harden user_project_wrapper"

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
[ -f "$WRAP_GDS" ] || die "wrapper streamout produced no GDS (rc=$WRC)"
log "  wrapper GDS produced (rc=$WRC, run=$WRAP_RUN)"

bash "$COPY" "$PROJ" user_project_wrapper "$WRAP_RUN" > "$LOG_DIR/copy_views_wrap.log" 2>&1 \
  || die "copy_views wrapper failed"
log "  wrapper views copied"

# Verify no dup-header malformation in wrapper (sanity)
python3 "$PROJ/scripts/gds_dedup.py" "$PROJ/gds/user_project_wrapper.gds" /tmp/wrap_dedup_check.gds 2>"$LOG_DIR/wrap_dedup_check.log"
DUP=$(grep -c "fixed duplicate header" "$LOG_DIR/wrap_dedup_check.log" 2>/dev/null || echo 0)
if [ "$DUP" -gt 0 ]; then
  log "  wrapper had $DUP dup-header(s); applying fix"
  cp /tmp/wrap_dedup_check.gds "$PROJ/gds/user_project_wrapper.gds"
else
  log "  wrapper GDS clean (no dup headers)"
fi
rm -f /tmp/wrap_dedup_check.gds

# -----------------------------------------------------------------------------
# Step 2: cf precheck
# -----------------------------------------------------------------------------
log "Step 2: cf precheck"
cd "$PROJ"
cf precheck > "$LOG_DIR/cf_precheck.log" 2>&1
PRC=$?
if [ $PRC -ne 0 ]; then
  die "cf precheck exit=$PRC (see $LOG_DIR/cf_precheck.log)"
fi
if grep -E "FAIL  " "$LOG_DIR/cf_precheck.log" >/dev/null 2>&1; then
  die "cf precheck reported FAIL lines (see $LOG_DIR/cf_precheck.log)"
fi
log "  cf precheck PASSED"

# -----------------------------------------------------------------------------
# Step 3: cf push
# -----------------------------------------------------------------------------
log "Step 3: cf push"
cd "$PROJ"
cf push > "$LOG_DIR/cf_push.log" 2>&1
PUSHRC=$?
if [ $PUSHRC -ne 0 ]; then
  die "cf push exit=$PUSHRC (see $LOG_DIR/cf_push.log)"
fi
log "  cf push completed"

sed -i '' 's/^RESULT=RUNNING/RESULT=SUCCESS/' "$STATUS" 2>/dev/null || echo "RESULT=SUCCESS" >> "$STATUS"
log "AUTOPILOT PLAN B DONE — re-spin submitted to ChipFoundry"
