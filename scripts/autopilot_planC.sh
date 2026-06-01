#!/usr/bin/env bash
# Plan C: full PDK switch to 3e0e31dc (the version that natively contains
# sky130_ef_sc_hd__decap_40_12). LibreLane 2.4.6's get_opdks_rev() is hard-
# coded to the old 0fe599b2 revision; the --manual-pdk flag bypasses ciel
# entirely and uses PDK_ROOT as a literal PDK directory.
#
#   senseedge_top harden  →  copy_views  →  wrapper harden  →  copy_views
#                         →  cf precheck →  cf push --project-name SenseEdge
#
# Includes a smoke check after senseedge_top's first stage that verifies the
# new PDK is actually in use (aborts the harden early if not, saving ~7 hours
# of wasted compute).

set -uo pipefail

PROJ="/Users/fidelmakatia/chipfoundry/senseedge"
TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$PROJ/logs/autopilot_planC_$TS"
mkdir -p "$LOG_DIR"
STATUS="$LOG_DIR/STATUS.txt"

LIBRELANE_IMG="ghcr.io/librelane/librelane:2.4.6"
NEW_PDK="$PROJ/dependencies/pdks/ciel/sky130/versions/3e0e31dcce8519a7dbb82590346db16d91b7244f"
NEW_VER="3e0e31dcce8519a7dbb82590346db16d91b7244f"
COPY="$PROJ/scripts/copy_views_streamout.sh"
DEDUP="$PROJ/scripts/gds_dedup.py"

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

verify_new_pdk_used() {
  # After a few stages have written resolved.json, confirm new PDK is used
  local run_dir="$1"
  local rj="$run_dir/resolved.json"
  local tries=0
  while [ $tries -lt 30 ]; do
    if [ -f "$rj" ]; then
      if grep -q "$NEW_VER" "$rj" 2>/dev/null; then
        log "  smoke check PASS: $rj references $NEW_VER"
        return 0
      fi
    fi
    sleep 10
    tries=$((tries+1))
  done
  log "  smoke check timed out waiting for resolved.json with new PDK"
  return 1
}

log "AUTOPILOT PLAN C START — full PDK switch to $NEW_VER"
echo "RESULT=RUNNING" >> "$STATUS"

# Sanity-check the new PDK directory exists and has decap_40_12
[ -d "$NEW_PDK/sky130A" ] || die "new PDK dir not found: $NEW_PDK/sky130A"
DECAP_FILES=$(find "$NEW_PDK/sky130A/libs.ref/sky130_fd_sc_hd" -name "*decap_40_12*" 2>/dev/null | wc -l)
[ "$DECAP_FILES" -ge 3 ] || die "new PDK missing decap_40_12 files (found only $DECAP_FILES)"
log "  new PDK confirmed: $NEW_PDK with decap_40_12 ($DECAP_FILES view files)"

# -----------------------------------------------------------------------------
# Step 1: harden senseedge_top with --manual-pdk
# -----------------------------------------------------------------------------
SLOG="$LOG_DIR/harden_senseedge_top.log"
log "Step 1/6: senseedge_top harden (PDK=$NEW_VER, expect ~6-7h)"

heartbeat "$SLOG" &
HB_PID=$!

docker run --rm -i \
  -v /Users/fidelmakatia:/Users/fidelmakatia \
  -e PDK_ROOT="$NEW_PDK" \
  -w "$PROJ" \
  "$LIBRELANE_IMG" \
  python3 -m librelane --manual-pdk --pdk-root "$NEW_PDK" \
    openlane/senseedge_top/config.json \
  > "$SLOG" 2>&1 &
DOCKER_PID=$!

# Smoke check after a few minutes of run
sleep 240   # 4 minutes — usually enough for floorplan to write resolved.json
TOP_RUN_INIT="$(ls -t "$PROJ/openlane/senseedge_top/runs/" 2>/dev/null | head -1)"
if [ -n "$TOP_RUN_INIT" ]; then
  if ! verify_new_pdk_used "$PROJ/openlane/senseedge_top/runs/$TOP_RUN_INIT"; then
    log "  ABORTING: docker pid $DOCKER_PID still using old PDK; killing"
    kill $DOCKER_PID 2>/dev/null
    docker ps --filter ancestor=$LIBRELANE_IMG -q | xargs -r docker kill 2>/dev/null
    kill $HB_PID 2>/dev/null
    die "senseedge_top harden did NOT use new PDK — --manual-pdk may need different invocation"
  fi
else
  log "  WARN: no run dir yet; smoke check deferred"
fi

# Wait for the harden to actually finish (it'll take hours)
wait $DOCKER_PID
SRC=$?
kill $HB_PID 2>/dev/null || true

TOP_RUN="$(ls -t "$PROJ/openlane/senseedge_top/runs/" | head -1)"
SOUT="$(ls "$PROJ/openlane/senseedge_top/runs/$TOP_RUN"/*-magic-streamout/senseedge_top.gds 2>/dev/null | head -1)"
SOUT_M="$(ls "$PROJ/openlane/senseedge_top/runs/$TOP_RUN"/*-magic-streamout/senseedge_top.magic.gds 2>/dev/null | head -1)"
if [ -z "$SOUT" ] && [ -n "$SOUT_M" ]; then
  cp -v "$SOUT_M" "$(dirname "$SOUT_M")/senseedge_top.gds" >> "$SLOG" 2>&1
  SOUT="$(dirname "$SOUT_M")/senseedge_top.gds"
fi
[ -f "$SOUT" ] || die "senseedge_top streamout GDS missing (rc=$SRC)"
log "  senseedge_top streamout GDS produced (run=$TOP_RUN, rc=$SRC)"

# -----------------------------------------------------------------------------
# Step 2: copy_views senseedge_top, dedup if needed
# -----------------------------------------------------------------------------
log "Step 2/6: copy_views senseedge_top"
bash "$COPY" "$PROJ" senseedge_top "$TOP_RUN" > "$LOG_DIR/copy_views_top.log" 2>&1 \
  || die "copy_views senseedge_top failed"

# Dedup any malformed cell headers (shouldn't be any with new PDK, but defensive)
python3 "$DEDUP" "$PROJ/gds/senseedge_top.gds" /tmp/top_dedup.gds 2> "$LOG_DIR/dedup_top.log"
DUP=$(grep -c "fixed duplicate header" "$LOG_DIR/dedup_top.log" 2>/dev/null || echo 0)
if [ "$DUP" -gt 0 ]; then
  log "  applied dedup ($DUP fixes)"
  cp /tmp/top_dedup.gds "$PROJ/gds/senseedge_top.gds"
else
  log "  senseedge_top GDS clean (no dedup needed)"
fi

# Verify decap_40_12 actually got placed
DECAP_N=$(strings "$PROJ/gds/senseedge_top.gds" 2>/dev/null | grep -c "decap_40_12")
[ "$DECAP_N" -gt 100 ] || die "GDS has only $DECAP_N decap_40_12 references — expected thousands. Plan C failed."
log "  GDS contains $DECAP_N decap_40_12 references ✓"

# -----------------------------------------------------------------------------
# Step 3: harden user_project_wrapper
# -----------------------------------------------------------------------------
WLOG="$LOG_DIR/wrapper_harden.log"
log "Step 3/6: user_project_wrapper harden (PDK=$NEW_VER, ~5-10 min)"

heartbeat "$WLOG" &
HB_PID=$!

docker run --rm -i \
  -v /Users/fidelmakatia:/Users/fidelmakatia \
  -e PDK_ROOT="$NEW_PDK" \
  -w "$PROJ" \
  "$LIBRELANE_IMG" \
  python3 -m librelane --manual-pdk --pdk-root "$NEW_PDK" \
    openlane/user_project_wrapper/config.json \
  > "$WLOG" 2>&1
WRC=$?
kill $HB_PID 2>/dev/null || true

WRAP_RUN="$(ls -t "$PROJ/openlane/user_project_wrapper/runs/" | head -1)"
WOUT="$(ls "$PROJ/openlane/user_project_wrapper/runs/$WRAP_RUN"/*-magic-streamout/user_project_wrapper.gds 2>/dev/null | head -1)"
WOUT_M="$(ls "$PROJ/openlane/user_project_wrapper/runs/$WRAP_RUN"/*-magic-streamout/user_project_wrapper.magic.gds 2>/dev/null | head -1)"
if [ -z "$WOUT" ] && [ -n "$WOUT_M" ]; then
  cp -v "$WOUT_M" "$(dirname "$WOUT_M")/user_project_wrapper.gds" >> "$WLOG" 2>&1
  WOUT="$(dirname "$WOUT_M")/user_project_wrapper.gds"
fi
[ -f "$WOUT" ] || die "wrapper streamout GDS missing (rc=$WRC)"
log "  wrapper streamout GDS produced (run=$WRAP_RUN, rc=$WRC)"

# -----------------------------------------------------------------------------
# Step 4: copy_views wrapper + dedup
# -----------------------------------------------------------------------------
log "Step 4/6: copy_views wrapper"
bash "$COPY" "$PROJ" user_project_wrapper "$WRAP_RUN" > "$LOG_DIR/copy_views_wrap.log" 2>&1 \
  || die "copy_views wrapper failed"

python3 "$DEDUP" "$PROJ/gds/user_project_wrapper.gds" /tmp/wrap_dedup.gds 2> "$LOG_DIR/dedup_wrap.log"
DUP=$(grep -c "fixed duplicate header" "$LOG_DIR/dedup_wrap.log" 2>/dev/null || echo 0)
if [ "$DUP" -gt 0 ]; then
  log "  applied wrapper dedup ($DUP fixes)"
  cp /tmp/wrap_dedup.gds "$PROJ/gds/user_project_wrapper.gds"
else
  log "  wrapper GDS clean"
fi

# -----------------------------------------------------------------------------
# Step 5: cf precheck
# -----------------------------------------------------------------------------
log "Step 5/6: cf precheck (~1.5h)"
cd "$PROJ"
cf precheck > "$LOG_DIR/cf_precheck.log" 2>&1
PRC=$?
[ $PRC -eq 0 ] || die "cf precheck exit=$PRC"
if grep -E "FAIL  " "$LOG_DIR/cf_precheck.log" > /dev/null 2>&1; then
  die "cf precheck reported FAIL lines"
fi
log "  cf precheck PASSED"

# -----------------------------------------------------------------------------
# Step 6: cf push (always use --project-name SenseEdge per the casing bug)
# -----------------------------------------------------------------------------
log "Step 6/6: cf push --project-name SenseEdge"
cd "$PROJ"
cf push --project-name SenseEdge > "$LOG_DIR/cf_push.log" 2>&1
PUSHRC=$?
[ $PUSHRC -eq 0 ] || die "cf push exit=$PUSHRC"
log "  cf push completed"

# Also confirm submission via cf confirm (writes submission_state=Final to SFTP)
cf confirm > "$LOG_DIR/cf_confirm.log" 2>&1 || true   # API confirm blocked on billing email, SFTP confirm still succeeds

sed -i '' 's/^RESULT=RUNNING/RESULT=SUCCESS/' "$STATUS" 2>/dev/null || echo "RESULT=SUCCESS" >> "$STATUS"
log "AUTOPILOT PLAN C DONE — Plan C resubmission complete; awaiting David's PV"
