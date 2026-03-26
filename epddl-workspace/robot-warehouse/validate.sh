#!/bin/bash
# =============================================================================
# validate.sh — Ejecuta parse, ground y validate para warehouse-inspection-1
# Requiere: plank (https://github.com/a-burigana/plank)
# Uso: bash validate.sh
# =============================================================================

LIB=~/plank/benchmarks/libraries/intermediate.epddl
DOMAIN=warehouse-domain.epddl
PROBLEM=warehouse-problem.epddl

echo "============================================================"
echo " PARSE"
echo "============================================================"
plank parse \
  -d $DOMAIN \
  -p $PROBLEM \
  -l $LIB

echo ""
echo "============================================================"
echo " GROUND"
echo "============================================================"
plank ground \
  -d $DOMAIN \
  -p $PROBLEM \
  -l $LIB

# =============================================================================
# Plan A: Scout va a Z3, inspecciona en privado, reporta al supervisor,
#         carrier recoge la pieza, supervisor activa alarma
# =============================================================================
echo ""
echo "============================================================"
echo " VALIDATE — Plan A (inspección + reporte + recogida + alarma)"
echo "============================================================"
plank validate \
  -d $DOMAIN \
  -p $PROBLEM \
  -l $LIB \
  -a \
    "move-Z1-to-Z2-arrive_r_scout" \
    "move-Z1-to-Z2-leave_r_scout" \
    "move-Z2-to-Z3-arrive_r_scout" \
    "move-Z2-to-Z3-leave_r_scout" \
    "inspect-private-Z3_r_scout" \
    "report-damage-Z3_r_scout_r_super" \
    "move-Z1-to-Z2-arrive_r_carrier" \
    "move-Z1-to-Z2-leave_r_carrier" \
    "move-Z2-to-Z3-arrive_r_carrier" \
    "move-Z2-to-Z3-leave_r_carrier" \
    "pickup-damaged-Z3-carry_r_carrier" \
    "pickup-damaged-Z3-remove_r_carrier" \
    "pickup-damaged-Z3-mark_r_carrier" \
    "raise-alarm-Z3_r_super"

# =============================================================================
# Plan B (inválido): carrier recoge sin reporte previo.
#         La meta epistémica ([r_super](piece-removed-Z3)) debe FALLAR.
# =============================================================================
echo ""
echo "============================================================"
echo " VALIDATE — Plan B (sin reporte: meta epistémica debe FALLAR)"
echo "============================================================"
plank validate \
  -d $DOMAIN \
  -p $PROBLEM \
  -l $LIB \
  -a \
    "move-Z1-to-Z2-arrive_r_carrier" \
    "move-Z1-to-Z2-leave_r_carrier" \
    "move-Z2-to-Z3-arrive_r_carrier" \
    "move-Z2-to-Z3-leave_r_carrier" \
    "pickup-damaged-Z3-carry_r_carrier" \
    "pickup-damaged-Z3-remove_r_carrier" \
    "pickup-damaged-Z3-mark_r_carrier" \
    "raise-alarm-Z3_r_super"

# =============================================================================
# Plan C: Scout también despeja el obstáculo de Z2 antes de ir a Z3
# =============================================================================
echo ""
echo "============================================================"
echo " VALIDATE — Plan C (scout despeja Z2 primero)"
echo "============================================================"
plank validate \
  -d $DOMAIN \
  -p $PROBLEM \
  -l $LIB \
  -a \
    "move-Z1-to-Z2-arrive_r_scout" \
    "move-Z1-to-Z2-leave_r_scout" \
    "clear-obstacle-Z2-remove_r_scout" \
    "clear-obstacle-Z2-mark_r_scout" \
    "move-Z2-to-Z3-arrive_r_scout" \
    "move-Z2-to-Z3-leave_r_scout" \
    "inspect-private-Z3_r_scout" \
    "report-damage-Z3_r_scout_r_super" \
    "move-Z1-to-Z2-arrive_r_carrier" \
    "move-Z1-to-Z2-leave_r_carrier" \
    "move-Z2-to-Z3-arrive_r_carrier" \
    "move-Z2-to-Z3-leave_r_carrier" \
    "pickup-damaged-Z3-carry_r_carrier" \
    "pickup-damaged-Z3-remove_r_carrier" \
    "pickup-damaged-Z3-mark_r_carrier" \
    "raise-alarm-Z3_r_super"