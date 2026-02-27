#!/bin/bash

echo "=== PARSE ==="
plank parse \
  -d box-task-domain.epddl \
  -p box-task-problem.epddl \
  -l ~/plank/benchmarks/libraries/intermediate.epddl

echo "=== GROUND ==="
plank ground \
  -d box-task-domain.epddl \
  -p box-task-problem.epddl \
  -l ~/plank/benchmarks/libraries/intermediate.epddl

echo "=== VALIDATE (2-step plan) ==="
plank validate \
  -d box-task-domain.epddl \
  -p box-task-problem.epddl \
  -l ~/plank/benchmarks/libraries/intermediate.epddl \
  -a "pickup-A-hold_r2" "pickup-A-clear_r2"


# Actualización epistémica:
#   R_r1 se restringe
#   R_r2 permanece igual
#
# Se vuelve a verificar:
#   M',w ⊨ K_r2(clear(A))

echo "=== VALIDATE (3-step plan with observe) ==="
plank validate \
  -d box-task-domain.epddl \
  -p box-task-problem.epddl \
  -l ~/plank/benchmarks/libraries/intermediate.epddl \
  -a "observe-private-A_r1" "pickup-A-hold_r2" "pickup-A-clear_r2"
