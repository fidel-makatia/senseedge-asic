#!/usr/bin/env bash
# Streamout-based view copier.
# LibreLane 2.4.6 doesn't write final/* when last-stage timing checkers
# emit deferred errors, so we source from the individual stage dirs.
# Also tolerates runs that aborted after Magic.StreamOut (no writelef stage)
# — the existing project LEF is kept in place since outline + pins don't
# change run-to-run.

set -uo pipefail

if [ $# -ne 3 ]; then
  echo "Usage: $0 PROJECT_ROOT MACRO RUN_TAG"
  exit 1
fi

PROJECT_ROOT="$1"
MACRO="$2"
RUN_TAG="$3"
RUN="$PROJECT_ROOT/openlane/$MACRO/runs/$RUN_TAG"

[ -d "$RUN" ] || { echo "ERROR: run dir not found: $RUN"; exit 1; }

mkdir -p "$PROJECT_ROOT/signoff/$MACRO/openlane-signoff/timing-reports"

stage() {
  # Find the directory matching pattern "NN-${pattern}" under $RUN.
  local pat="$1"
  ls -d "$RUN"/*-"$pat" 2>/dev/null | head -1
}

ST_STREAMOUT="$(stage magic-streamout)"
ST_WRITELEF="$(stage magic-writelef)"
ST_DETROUTE="$(stage openroad-detailedrouting)"
ST_RCX="$(stage openroad-rcx)"
ST_STAPOSTPNR="$(stage openroad-stapostpnr)"
ST_MAGIC_DRC="$(stage magic-drc)"
ST_LVS="$(stage netgen-lvs)"

[ -n "$ST_STREAMOUT" ] || { echo "ERROR: no magic-streamout stage in $RUN"; exit 1; }

echo "==> copy $MACRO views from $RUN"

# GDS (250 MB-ish). Magic writes both $MACRO.gds and $MACRO.magic.gds;
# if only the magic.gds form exists (e.g. flow aborted right after streamout),
# fall back to that.
if [ -f "$ST_STREAMOUT/$MACRO.gds" ]; then
  cp -v "$ST_STREAMOUT/$MACRO.gds" "$PROJECT_ROOT/gds/$MACRO.gds"
elif [ -f "$ST_STREAMOUT/$MACRO.magic.gds" ]; then
  cp -v "$ST_STREAMOUT/$MACRO.magic.gds" "$PROJECT_ROOT/gds/$MACRO.gds"
else
  echo "ERROR: no GDS in $ST_STREAMOUT"; exit 1
fi

# mag
if [ -f "$ST_STREAMOUT/$MACRO.mag" ]; then
  mkdir -p "$PROJECT_ROOT/mag"
  cp -v "$ST_STREAMOUT/$MACRO.mag" "$PROJECT_ROOT/mag/$MACRO.mag"
fi

# LEF — if this run didn't reach magic-writelef stage, keep the existing
# project LEF (outline + pins are determined by DIE_AREA + FP_PIN_ORDER_CFG,
# neither of which changes between runs)
if [ -n "$ST_WRITELEF" ] && [ -f "$ST_WRITELEF/$MACRO.lef" ]; then
  cp -v "$ST_WRITELEF/$MACRO.lef" "$PROJECT_ROOT/lef/$MACRO.lef"
else
  echo "(skipping LEF copy — writelef stage absent; keeping existing $PROJECT_ROOT/lef/$MACRO.lef)"
fi

# Gate-level Verilog (use the post-detailed-route netlist with power pins)
if [ -n "$ST_DETROUTE" ] && [ -f "$ST_DETROUTE/$MACRO.pnl.v" ]; then
  mkdir -p "$PROJECT_ROOT/verilog/gl"
  cp -v "$ST_DETROUTE/$MACRO.pnl.v" "$PROJECT_ROOT/verilog/gl/$MACRO.v"
fi

# SPEF (multicorner + nominal)
if [ -n "$ST_RCX" ]; then
  mkdir -p "$PROJECT_ROOT/spef/multicorner"
  for c in min nom max; do
    SRC=$(ls "$ST_RCX/$c/"*.spef 2>/dev/null | head -1)
    if [ -n "$SRC" ]; then
      cp -v "$SRC" "$PROJECT_ROOT/spef/multicorner/$MACRO.$c.spef"
    fi
  done
  # Nominal as the "default" .spef
  if [ -f "$PROJECT_ROOT/spef/multicorner/$MACRO.nom.spef" ]; then
    mkdir -p "$PROJECT_ROOT/spef"
    cp -v "$PROJECT_ROOT/spef/multicorner/$MACRO.nom.spef" "$PROJECT_ROOT/spef/$MACRO.spef"
  fi
fi

# LIB — use nominal typical corner
if [ -n "$ST_STAPOSTPNR" ]; then
  LIB_SRC=$(ls "$ST_STAPOSTPNR/nom_tt_025C_1v80/"*.lib 2>/dev/null | head -1)
  if [ -n "$LIB_SRC" ]; then
    cp -v "$LIB_SRC" "$PROJECT_ROOT/lib/$MACRO.lib"
  fi
fi

# DRC + LVS reports to signoff
if [ -n "$ST_MAGIC_DRC" ] && [ -d "$ST_MAGIC_DRC/reports" ]; then
  cp -r "$ST_MAGIC_DRC/reports/." "$PROJECT_ROOT/signoff/$MACRO/openlane-signoff/" || true
fi
if [ -n "$ST_LVS" ]; then
  [ -d "$ST_LVS/reports" ] && cp -r "$ST_LVS/reports/." "$PROJECT_ROOT/signoff/$MACRO/openlane-signoff/" || true
  [ -f "$ST_LVS/netgen-lvs.log" ] && cp "$ST_LVS/netgen-lvs.log" "$PROJECT_ROOT/signoff/$MACRO/openlane-signoff/" || true
fi

# Resolved config snapshot
[ -f "$RUN/resolved.json" ] && cp "$RUN/resolved.json" "$PROJECT_ROOT/signoff/$MACRO/" || true

echo "==> copy $MACRO views: done"
