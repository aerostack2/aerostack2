#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"

export XML_CATALOG_FILES="${workspace_dir}/tools/ros_schema/catalog.xml"
if [[ -n "${CONDA_PREFIX:-}" && -f "${CONDA_PREFIX}/setup.bash" ]]; then
  source "${CONDA_PREFIX}/setup.bash"
fi
source "${workspace_dir}/install/local_setup.bash"

if [[ "${1:-}" == "--result" ]]; then
  shift
  exec colcon test-result --verbose "$@"
fi

exec colcon test \
  --event-handlers console_direct+ \
  --parallel-workers "${COLCON_TEST_WORKERS:-1}" \
  "$@"
