#!/usr/bin/env python3
"""Plan which colcon packages a CI run builds, and how to split them into jobs.

Every package gets its own job. The packages listed in --base (the ones most
others depend on) run first, in stages that follow their dependencies: a stage
starts once the stages before it finish, and reuses their builds. The other
packages run last, all in parallel, on top of the base builds.

Packages from other repositories (--external, e.g. the aerial platforms) are
cloned and planned like the rest: they get their own job when selected, which
happens when something they depend on changed.

Selection:
  --all      every package in the workspace (pushes, manual runs)
  --diff REF packages changed since the merge-base with REF, plus everything
             that depends on them. Changes outside any package that affect the
             whole build (CI files, root build files) select every package.
             Changes that touch no package (docs) select nothing.

Writes these GitHub Actions outputs:
  skip        'true' when nothing needs building
  base        base packages to build (only those the selection needs)
  stage1..N   JSON list of {"package", "test"} for each base stage. Base
              packages are tested only when selected.
  packages    JSON list of {"package", "repo"} for the remaining packages, one
              job each. "repo" is the owner/name to clone for external
              packages, empty for packages in this repository.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import tempfile
from pathlib import Path, PurePosixPath

# Number of base stages the workflow defines (jobs stage1..stageN).
STAGES = 3

# Changes under these paths can affect every package.
FULL_BUILD_PREFIXES = (".github/", "aerostack2/")
FULL_BUILD_FILES = ("pixi.toml", "pixi.lock")


# Folders colcon searches for packages: this repository plus the clones of
# external repositories.
BASE_PATHS = ["."]


def colcon_list(*args: str) -> set[str]:
    out = subprocess.run(
        ["colcon", "list", "--names-only", "--base-paths", *BASE_PATHS, *args],
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


def clone_external(repos: list[str]) -> dict[str, str]:
    """Clone each owner/name repository and map its packages to it."""
    if not repos:
        return {}
    root = Path(tempfile.mkdtemp(prefix="external-"))
    BASE_PATHS.append(str(root))
    owner = {}
    for repo in repos:
        dest = root / repo.replace("/", "__")
        git("clone", "--quiet", "--depth", "1", f"https://github.com/{repo}.git", str(dest))
        for pkg in subprocess.run(
            ["colcon", "list", "--names-only", "--base-paths", str(dest)],
            check=True, capture_output=True, text=True,
        ).stdout.split():
            owner[pkg] = repo
    return owner


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
    parser.add_argument(
        "--external", default="", help="space-separated owner/name repositories to also build"
    )
    args = parser.parse_args()

    external = clone_external(args.external.split())
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

    # Stage of a base package: one after the latest stage of its base deps.
    stage: dict[str, int] = {}

    def stage_of(pkg: str) -> int:
        if pkg not in stage:
            deps = colcon_list("--packages-up-to", pkg) & base_build - {pkg}
            stage[pkg] = 1 + max((stage_of(d) for d in deps), default=0)
        return stage[pkg]

    for pkg in base_build:
        stage_of(pkg)
    if stage and max(stage.values()) > STAGES:
        raise SystemExit(
            f"base packages need {max(stage.values())} stages, the workflow has {STAGES}"
        )

    outputs = {
        "skip": "true" if not selected else "false",
        "base": " ".join(sorted(base_build)),
    }
    for n in range(1, STAGES + 1):
        jobs = [
            {"package": pkg, "test": pkg in selected}
            for pkg in sorted(base_build) if stage[pkg] == n
        ]
        outputs[f"stage{n}"] = json.dumps(jobs)
    outputs["packages"] = json.dumps([
        {"package": pkg, "repo": external.get(pkg, "")} for pkg in sorted(selected - base)
    ])
    lines = [f"{key}={value}" for key, value in outputs.items()]
    print("\n".join(lines))
    if github_output := os.environ.get("GITHUB_OUTPUT"):
        with open(github_output, "a", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
