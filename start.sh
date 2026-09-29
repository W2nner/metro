#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
exec python3 -m metro_detector serve --state state "$@"
