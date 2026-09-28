#!/usr/bin/env bash
# Ubuntu 22.04 x86_64 fallback when system-wide sudo installation is unavailable.
set -euo pipefail
prefix="${GCOPTER_DEPS_PREFIX:-$HOME/.local/share/gcopter-ros2-deps/usr}"
mkdir -p "${prefix%/usr}/downloads"
cd "${prefix%/usr}/downloads"
apt-get download libompl-dev libompl16
for package in libompl-dev_*.deb libompl16_*.deb; do
  dpkg-deb -x "$package" "${prefix%/usr}"
done
printf 'OMPL extracted to %s; source scripts/env.sh before building/running.\n' "$prefix"
