#!/usr/bin/env bash
# Build one package with pixi for one ROS distro. If the build fails, show
# just the errors, also in the GitHub job summary.
#
# Usage: build.sh <distro> <package-folder> [output-dir]
#
# pixi build always uses the channels of the "default" environment, and the
# ROS distro follows from those channels. There's no option to pick another
# environment, so the distro's feature is made the default environment first
# (the same trick pixi-release.yaml uses). This edits pixi.toml in the
# checkout, which is fine in CI.
set -uo pipefail

distro=$1
package_path=$2
output_dir=${3:-${RUNNER_TEMP:-/tmp}/pixi-packages}
log=$(mktemp)
summary=${GITHUB_STEP_SUMMARY:-/dev/null}

pixi workspace environment add default --feature "$distro" --force || exit 1
pixi build --path "$package_path" --output-dir "$output_dir" 2>&1 | tee "$log"
status=${PIPESTATUS[0]}
[ "$status" -eq 0 ] && exit 0

# The log is long (thousands of lines), colored, and every line is prefixed
# with pixi's box drawing. Strip those, then pull out the errors.
clean=$(mktemp)
sed -e 's/\x1b\[[0-9;]*m//g' -e 's/^ *│ │ //' "$log" > "$clean"

# 1. The distinct error lines, e.g. "src/foo.cpp:12:3: error: ...".
errors=$(grep -E ': (fatal )?error:|undefined reference|CMake Error' "$clean" | sort -u)

# 2. The full output of each file that failed to compile or link, without the
#    compiler command line that ninja prints first. Without ninja failures
#    (e.g. a CMake or dependency error), the end of the log.
if grep -q '^FAILED:' "$clean"; then
  # On a FAILED line: print it, skip the compiler command after it (pixi may
  # wrap it over several lines) until the first compiler or linker message,
  # then keep printing until ninja stops.
  details=$(awk '
    /^FAILED:/              { print; started = 1; skipping = 1; next }
    /^ninja: build stopped/ { exit }
    skipping && /^In file included from |:[0-9]+:[0-9]+: |: (fatal )?error|undefined reference|^ld: |^collect2/ { skipping = 0 }
    started && !skipping
  ' "$clean")
elif grep -q 'CMake Error' "$clean"; then
  details=$(grep -A 30 -m 1 'CMake Error' "$clean")
else
  details=$(tail -n 40 "$clean")
fi

echo "::group::Errors in $package_path ($distro)"
echo "${errors:-No compiler errors found; see the details below.}"
echo
echo "$details"
echo "::endgroup::"
{
  echo "### ❌ pixi build failed: \`$package_path\` ($distro)"
  echo '```'
  echo "${errors:-No compiler errors found; see the details below.}"
  echo '```'
  echo '<details><summary>Details</summary>'
  echo
  echo '```'
  echo "$details" | head -n 300
  echo '```'
  echo '</details>'
} >> "$summary"
exit "$status"
