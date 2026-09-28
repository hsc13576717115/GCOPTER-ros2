#!/usr/bin/env bash
# Source this file from bash. Optional local OMPL is outside the repository.
# Snap-packaged IDEs may inject incompatible GTK/GIO libraries into native Qt apps.
if [[ "${GTK_PATH:-}" == /snap/* || "${GTK_EXE_PREFIX:-}" == /snap/* ]]; then
  unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR
  if [ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]; then
    export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
  fi
fi
source /opt/ros/humble/setup.bash
_gcopter_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_gcopter_deps="${GCOPTER_DEPS_PREFIX:-$HOME/.local/share/gcopter-ros2-deps/usr}"
if [ -d "$_gcopter_deps/share/ompl/cmake" ]; then
  export CMAKE_PREFIX_PATH="$_gcopter_deps${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$_gcopter_deps/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
if [ -f "$_gcopter_root/install/setup.bash" ]; then
  source "$_gcopter_root/install/setup.bash"
fi
unset _gcopter_root _gcopter_deps
