#!/usr/bin/env bash
# Backward-compatible entry point. The maintained collector performs the
# controller-load handshake, safe Gazebo unpause, gait switching, and logging.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/collect_trot_amp_data.sh" "$@"
