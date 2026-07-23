#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
PROFILE_TOOL="$REPO_ROOT/tools/fixedwing/ucc_fixedwing_mvp_v1/01_apply_external_physics_profile.sh"

exec "$PROFILE_TOOL" \
  --profile runtime \
  --spawn-altitude-m 400 \
  --camera-pitch-deg -45 \
  --observer-follow-distance-m -6 \
  "$@"
