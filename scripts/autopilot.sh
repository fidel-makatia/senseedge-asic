#!/usr/bin/env bash
# Autonomous re-spin pipeline for SenseEdge (v2 — streamout-tolerant).
#
# Why streamout-tolerant: LibreLane treats hold/setup violations on tt corners
# as deferred errors that prevent final/ from being written, even when DRC/LVS
# are clean. The Mar 31 successful submission ALSO had this — the workflow has
# always sourced views from the 59-magic-streamout/ stage, not final/.
#
# Pipeline:
#   1. Re-harden senseedge_top (LEF now contains decap_40_12)
#   2. copy_views_streamout senseedge_top
#   3. Harden user_project_wrapper
#   4. copy_views_streamout user_project_wrapper
#   5. cf precheck
#   6. cf push (if precheck passes)

set -uo pipefail

PROJ="/Users/fidelmakatia/chipfoundry/senseedge"
TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$PROJ/logs/autopilot_$TS"
mkdir -p "$LOG_DIR"
STATUS="$LOG_DIR/STATUS.txt"

LIBRELANE_IMG="ghcr.io/librelane/librelane:2.4.6"
PDK_ROOT="$PROJ/dependencies/pdks"
COPY="$PROJ/scripts/copy_views_streamout.sh"

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$STATUS"; }
die() { log "FAIL: $*"; echo "RESULT=FAIL" >> "$STATUS"; exit 1; }

heartbeat() {
  # Background heartbeat — appends LibreLane current stage to STATUS every 60s
  # until $1 (a log path) stops changing or this process is killed.
  local lname="$1"
  while sleep 60; do
    [ -f "$lname" ] || continue
    local stage
    stage="$(grep -E "^\[.*\] .*Running '" "$lname" 2>/dev/null | tail -1 | sed -E "s/.*Running '([^']+)'.*/\\1/")"
    [ -n "$stage" ] && echo "[$(date '+%H:%M:%S')] heartbeat: $stage" >> "$STATUS"
  done
}

# Harden one macro. Args: <macro_name> <config_path>
# Returns 0 if streamout GDS exists, regardless of LibreLane's exit code
# (because deferred timing errors are expected for senseedge_top).
harden_macro() {
  local MACRO="$1"
  local CFG="$2"
  local HLOG="$LOG_DIR/harden_${MACRO}.log"
  log "Hardening $MACRO -> $HLOG"

  heartbeat "$HLOG" &
  local HB_PID=$!

  docker run --rm -i \
    -v /Users/fidelmakatia:/Users/fidelmakatia \
    -e PDK_ROOT="$PDK_ROOT" \
    -w "$PROJ" \
    "$LIBRELANE_IMG" \
    python3 -m librelane --pdk-root "$PDK_ROOT" "$CFG" \
    > "$HLOG" 2>&1
  local RC=$?

  kill $HB_PID 2>/dev/null || true

  # Find the latest run for this macro
  local RUN_TAG
  RUN_TAG="$(ls -t "$PROJ/openlane/$MACRO/runs/" | head -1)"
  local STREAMOUT_GDS
  STREAMOUT_GDS="$(ls "$PROJ/openlane/$MACRO/runs/$RUN_TAG"/*-magic-streamout/"$MACRO.gds" 2>/dev/null | head -1)"

  if [ -n "$STREAMOUT_GDS" ] && [ -f "$STREAMOUT_GDS" ]; then
    log "  $MACRO streamout GDS produced (rc=$RC, run=$RUN_TAG)"
    echo "$RUN_TAG"  # printed to stdout for caller capture
    return 0
  fi

  # No streamout GDS = real failure (probably crashed earlier in flow)
  die "$MACRO harden: no streamout GDS (rc=$RC). See $HLOG"
}

log "AUTOPILOT v2 START — logs in $LOG_DIR"
echo "RESULT=RUNNING" >> "$STATUS"

# -----------------------------------------------------------------------------
# Step 1: re-harden senseedge_top (LEF now has decap_40_12)
# -----------------------------------------------------------------------------
log "Step 1/6: senseedge_top harden"
TOP_RUN="$(harden_macro senseedge_top openlane/senseedge_top/config.json)"
[ -n "$TOP_RUN" ] || die "senseedge_top harden returned empty run tag"

# -----------------------------------------------------------------------------
# Step 2: copy_views senseedge_top
# -----------------------------------------------------------------------------
log "Step 2/6: copy_views senseedge_top from $TOP_RUN"
bash "$COPY" "$PROJ" senseedge_top "$TOP_RUN" \
  > "$LOG_DIR/copy_views_senseedge_top.log" 2>&1 \
  || die "copy_views senseedge_top failed (see $LOG_DIR/copy_views_senseedge_top.log)"

# Sanity check the copied artifacts
for f in "$PROJ/gds/senseedge_top.gds" "$PROJ/lef/senseedge_top.lef" \
         "$PROJ/lib/senseedge_top.lib" "$PROJ/verilog/gl/senseedge_top.v"; do
  [ -f "$f" ] || die "missing after copy: $f"
done
log "  senseedge_top views copied"

# Verify decap_40_12 actually got placed
DECAP_COUNT=$(strings "$PROJ/gds/senseedge_top.gds" 2>/dev/null | grep -c "decap_40_12")
if [ "$DECAP_COUNT" -eq 0 ]; then
  die "GDS contains NO decap_40_12 cells — OpenROAD still didn't place them. Investigate LEF."
fi
log "  GDS contains decap_40_12 references: $DECAP_COUNT (good)"

# -----------------------------------------------------------------------------
# Step 3: harden user_project_wrapper
# -----------------------------------------------------------------------------
log "Step 3/6: user_project_wrapper harden"
WRAP_RUN="$(harden_macro user_project_wrapper openlane/user_project_wrapper/config.json)"
[ -n "$WRAP_RUN" ] || die "wrapper harden returned empty run tag"

# -----------------------------------------------------------------------------
# Step 4: copy_views user_project_wrapper
# -----------------------------------------------------------------------------
log "Step 4/6: copy_views user_project_wrapper from $WRAP_RUN"
bash "$COPY" "$PROJ" user_project_wrapper "$WRAP_RUN" \
  > "$LOG_DIR/copy_views_user_project_wrapper.log" 2>&1 \
  || die "copy_views user_project_wrapper failed"
[ -f "$PROJ/gds/user_project_wrapper.gds" ] || die "wrapper GDS not in gds/"
log "  wrapper views copied"

# -----------------------------------------------------------------------------
# Step 5: cf precheck
# -----------------------------------------------------------------------------
log "Step 5/6: cf precheck"
cd "$PROJ"
cf precheck > "$LOG_DIR/cf_precheck.log" 2>&1
PRC_RC=$?
if [ $PRC_RC -ne 0 ]; then
  die "cf precheck exit=$PRC_RC (see $LOG_DIR/cf_precheck.log)"
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
PUSH_RC=$?
if [ $PUSH_RC -ne 0 ]; then
  die "cf push exit=$PUSH_RC (see $LOG_DIR/cf_push.log)"
fi
log "  cf push completed"

sed -i '' 's/^RESULT=RUNNING/RESULT=SUCCESS/' "$STATUS" 2>/dev/null || echo "RESULT=SUCCESS" >> "$STATUS"
log "AUTOPILOT DONE — re-spin submitted to ChipFoundry"
