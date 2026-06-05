#!/usr/bin/env bash
set -euo pipefail

WS="${VENOM_WS:-$HOME/venom_ws}"
NAV2_PARAMS="${NAV2_PARAMS:-$WS/src/venom_vnv/venom_bringup/config/scout_mini/nav2_teb_params.yaml}"

export NAV2_PARAMS
exec "$WS/src/venom_vnv/venom_bringup/scripts/start_mid360_nav2.sh"
