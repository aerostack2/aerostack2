#!/usr/bin/env python3
"""Plan which colcon packages a CI run builds, and how to split them into jobs.

The packages listed in --base are built once in a shared "base" job. Every
other selected package gets its own job, which reuses the base build.

Selection:
  --all      every package in the workspace (pushes, manual runs)
  --diff REF packages changed since the merge-base with REF, plus everything
             that depends on them. Changes outside any package that affect the
             whole build (CI files, root build files) select every package.
             Changes that touch no package (docs) select nothing.

Writes these GitHub Actions outputs:
  skip        'true' when nothing needs building
  base        base packages to build (only those the selection needs)
  base_test   base packages to test (only the selected ones)
  packages    JSON list with one entry per package job
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
from pathlib import PurePosixPath

# Changes under these paths can affect every package.
FULL_BUILD_PREFIXES = (".github/", "aerostack2/")
FULL_BUILD_FILES = ("pixi.toml", "pixi.lock")


def colcon_list(*args: str) -> set[str]:
    out = subprocess.run(
        ["colcon", "list", "--names-only", *args],
        check=True, capture_output=True, text=True,
    ).stdout
    return set(out.split())


def git(*args: str) -> str:
    return subprocess.run(["git", *args], check=True, capture_output=True, text=True).stdout


def package_paths() -> dict[PurePosixPath, str]:
    """Map each package folder (relative to the repo root) to its name."""
    out = subprocess.run(
        ["colcon", "list"], check=True, capture_output=True, text=True,
    ).stdout
    paths = {}
    for line in out.splitlines():
        name, path, _ = line.split("\t")
        paths[PurePosixPath(path)] = name
    return paths


def package_of(path: str, paths: dict[PurePosixPath, str]) -> str | None:
    """Return the package a file belongs to (the deepest folder that is one)."""
    for parent in PurePosixPath(path).parents:
        if parent in paths:
            return paths[parent]
    return None


def changed_packages(base_ref: str) -> set[str] | None:
    """Return the changed packages, or None when everything must be built."""
    merge_base = git("merge-base", base_ref, "HEAD").strip()
    changed = git("diff", "--name-only", merge_base, "HEAD").splitlines()
    if any(p.startswith(FULL_BUILD_PREFIXES) or p in FULL_BUILD_FILES for p in changed):
        return None
    paths = package_paths()
    return {pkg for p in changed if (pkg := package_of(p, paths))}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--all", action="store_true")
    mode.add_argument("--diff", metavar="REF")
    parser.add_argument("--base", default="", help="space-separated base packages")
    parser.add_argument("--exclude", default="", help="space-separated packages to never build")
    args = parser.parse_args()

    workspace = colcon_list()
    exclude = set(args.exclude.split())

    changed = None if args.all else changed_packages(args.diff)
    if changed is None:
        selected = workspace
    elif changed:
        selected = colcon_list("--packages-above", *sorted(changed))
    else:
        selected = set()
    selected -= exclude

    base = set(args.base.split()) & workspace
    unknown = set(args.base.split()) - workspace
    if unknown:
        raise SystemExit(f"base packages not in the workspace: {' '.join(sorted(unknown))}")

    # Build only the base packages the selection depends on (or tests).
    needed = colcon_list("--packages-up-to", *sorted(selected)) if selected else set()
    base_build = base & needed

    outputs = {
        "skip": "true" if not selected else "false",
        "base": " ".join(sorted(base_build)),
        "base_test": " ".join(sorted(base & selected)),
        "packages": json.dumps(sorted(selected - base)),
    }
    lines = [f"{key}={value}" for key, value in outputs.items()]
    print("\n".join(lines))
    if github_output := os.environ.get("GITHUB_OUTPUT"):
        with open(github_output, "a", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
