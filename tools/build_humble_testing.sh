#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"

export CC="${CC:-x86_64-conda-linux-gnu-gcc}"
export CXX="${CXX:-x86_64-conda-linux-gnu-g++}"

exec colcon build \
  --symlink-install \
  --parallel-workers "${COLCON_BUILD_WORKERS:-1}" \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCMAKE_C_COMPILER="${CC}" \
    -DCMAKE_CXX_COMPILER="${CXX}" \
  "$@"
