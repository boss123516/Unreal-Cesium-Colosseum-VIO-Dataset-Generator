#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"

exec "$REPO_ROOT/tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/02a_open_gazebo_follow_gui.sh" "$@"
