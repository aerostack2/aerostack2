#!/usr/bin/env python3
"""
Plan which colcon packages a CI run builds and tests: a root job, then one job
per package.

Selection
  Only packages with changed files are tested, plus every package that depends
  on them (colcon list --packages-above). What a change is compared against
  depends on the GitHub event:
    pull_request   the target branch (from the merge-base)
    push           the previous commit (event "before")
    anything else  nothing: every package is selected
  A push whose previous commit isn't in the history (new branch, force push)
  also selects every package. So do changes to this CI's own files (--ci-files),
  so CI changes get tested. Files outside any package (docs, other workflows)
  and files named in --ignore-files (e.g. pixi.toml for a colcon build) select
  nothing.

Jobs
  root      builds, once, every package that two or more of the package jobs
            would otherwise each build (usually as2_msgs and as2_core), and
            tests the selected ones among them. Skipped when nothing is shared.
  packages  one job per remaining selected package, all in parallel, on top of
            the root build. Each builds what its package needs that the root
            didn't build, then builds and tests it.

  Packages from other repositories (--external owner/name) are cloned and
  planned like the rest: they run when something they depend on changes.

Outputs (written to $GITHUB_OUTPUT, and printed)
  skip        'true' when nothing needs building
  root-build  packages the root job builds (space-separated, may be empty)
  root-test   packages the root job tests (space-separated, may be empty)
  packages    JSON list of jobs, one per package. Each job is {"package",
            "path", "repo"}: the package to build and test, its folder in
            this repository, and the owner/name repository to clone it from.
            For packages in this repository "repo" is empty; for external
            packages "path" is empty.

Run locally with --all or --diff REF to skip reading the GitHub event.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from collections import Counter
from pathlib import Path, PurePosixPath

# Folders colcon searches for packages: this repository plus the clones of
# external repositories.
BASE_PATHS = ["."]


def run(*args: str) -> str:
    """
    Run a command and return its standard output.

    :param args: The command and its arguments.
    :return: The command's standard output.
    :raises subprocess.CalledProcessError: If the command exits with a non-zero code.
    """
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def colcon_list(*args: str) -> set[str]:
    """
    List package names with ``colcon list``, searching every folder in BASE_PATHS.

    :param args: Extra ``colcon list`` arguments, usually a package selection such as
        ``--packages-above PKG`` or ``--packages-up-to PKG``.
    :return: The names of the packages colcon selects.
    """
    return set(run("colcon", "list", "--names-only", "--base-paths", *BASE_PATHS, *args).split())


def commit_exists(ref: str) -> bool:
    """
    Check whether a ref points to a commit in the local history.

    :param ref: A commit SHA, branch or other git ref.
    :return: True if the commit is available locally, False otherwise.
    """
    return subprocess.run(
        ["git", "cat-file", "-e", f"{ref}^{{commit}}"], capture_output=True
    ).returncode == 0


def diff_base_from_event() -> str | None:
    """
    Pick the ref to compare against from the GitHub event that started the run.

    Pull requests compare against their target branch. Pushes compare against
    the commit the branch pointed to before the push, when that commit is in
    the local history (it isn't for new branches or force pushes).

    :return: The ref to diff against, or None when every package must be selected.
    """
    event = os.environ.get("GITHUB_EVENT_NAME", "")
    if event in ("pull_request", "pull_request_target"):
        return f"origin/{os.environ['GITHUB_BASE_REF']}"
    if event == "push":
        with open(os.environ["GITHUB_EVENT_PATH"], encoding="utf-8") as fh:
            before = json.load(fh).get("before", "")
        if before and commit_exists(before):
            return before
    return None


def package_paths() -> dict[PurePosixPath, str]:
    """
    Map each package folder in this repository to its package name.

    :return: Package folders, relative to the repository root, mapped to names.
    """
    paths = {}
    for line in run("colcon", "list").splitlines():
        name, path, _ = line.split("\t")
        paths[PurePosixPath(path)] = name
    return paths


def package_of(path: str, paths: dict[PurePosixPath, str]) -> str | None:
    """
    Find the package a file belongs to.

    Walks up from the file to the closest folder that is a package, so files
    in nested packages map to the innermost one.

    :param path: File path relative to the repository root.
    :param paths: Package folders mapped to names, from package_paths().
    :return: The package name, or None if the file isn't inside any package.
    """
    for parent in PurePosixPath(path).parents:
        if parent in paths:
            return paths[parent]
    return None


def changed_packages(
    base_ref: str, ci_files: list[str], ignore_files: list[str]
) -> set[str] | None:
    """
    Find the packages with files changed since a ref.

    Compares HEAD against its merge-base with the ref, so a pull request only
    sees its own changes. Files outside any package are skipped, and so are
    files named in ignore_files.

    :param base_ref: The ref to compare against.
    :param ci_files: Paths or path prefixes whose changes select every package.
    :param ignore_files: File names whose changes affect no package (e.g. ``pixi.toml``).
    :return: The names of the changed packages (possibly empty), or None when a CI file changed
        and every package must be selected.
    """
    merge_base = run("git", "merge-base", base_ref, "HEAD").strip()
    changed = run("git", "diff", "--name-only", merge_base, "HEAD").splitlines()
    print(f"Changed files since {merge_base[:8]}:", *changed, sep="\n  ")
    if ci_files and any(p.startswith(tuple(ci_files)) for p in changed):
        print("CI files changed: selecting every package")
        return None
    paths = package_paths()
    return {
        pkg for p in changed
        if PurePosixPath(p).name not in ignore_files and (pkg := package_of(p, paths))
    }


def clone_external(repos: list[str]) -> dict[str, str]:
    """
    Clone external repositories so colcon can plan their packages.

    Each repository is shallow-cloned into a temporary folder, which is added
    to BASE_PATHS so later colcon_list() calls see its packages.

    :param repos: GitHub repositories as ``owner/name``.
    :return: Each package found in the clones mapped to the repository it came from.
    """
    if not repos:
        return {}
    root = Path(tempfile.mkdtemp(prefix="external-"))
    BASE_PATHS.append(str(root))
    owner = {}
    for repo in repos:
        dest = root / repo.replace("/", "__")
        run("git", "clone", "--quiet", "--depth", "1", f"https://github.com/{repo}.git", str(dest))
        for pkg in run("colcon", "list", "--names-only", "--base-paths", str(dest)).split():
            owner[pkg] = repo
    return owner


def split_root(selected: set[str]) -> set[str]:
    """
    Find the packages to build once in the root job, before the package jobs.

    Each selected package's job would build it together with everything it
    depends on. A package that two or more of those jobs would build is built
    once in the root job instead.

    :param selected: Packages that are built and tested.
    :return: The packages the root job builds, selected or not.
    """
    builds = Counter(
        dep for pkg in sorted(selected) for dep in colcon_list("--packages-up-to", pkg)
    )
    return {pkg for pkg, count in builds.items() if count > 1}


def names(packages: set[str]) -> str:
    """
    Format package names for display.

    :param packages: Package names.
    :return: The sorted names joined by commas, or "none".
    """
    return ", ".join(sorted(packages)) or "none"


def write_summary(
    mode: str, changed: set[str] | None, selected: set[str], root: set[str]
):
    """
    Print the plan and add it to the job summary on the run page.

    :param mode: What the change was compared against, for display.
    :param changed: Packages with changed files, or None for a full build.
    :param selected: Packages that are built and tested.
    :param root: Packages the root job builds.
    """
    lines = [
        "### Package plan",
        f"- **Compared against:** {mode}",
        "- **Changed packages:** " + ("all (full build)" if changed is None else names(changed)),
        f"- **Built and tested ({len(selected)}):** " + names(selected),
        f"- **Root job builds ({len(root)}):** " + names(root),
        f"- **Root job tests ({len(root & selected)}):** " + names(root & selected),
        f"- **Package jobs ({len(selected - root)}):** " + names(selected - root),
    ]
    print("\n".join(lines))
    if summary := os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(summary, "a", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")


def main() -> None:
    """
    Plan the CI run and write the job lists as GitHub Actions outputs.

    Options default to the action's inputs (INPUT_* environment variables),
    so the same script runs in CI and locally.

    :raises SystemExit: If the repository is a shallow clone.
    """
    env = os.environ.get
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--all", action="store_true", help="select every package")
    mode.add_argument("--diff", metavar="REF", help="select packages changed since REF")
    parser.add_argument("--exclude", default=env("INPUT_EXCLUDE_PACKAGES", ""))
    parser.add_argument("--external", default=env("INPUT_EXTERNAL_REPOS", ""))
    parser.add_argument("--ci-files", default=env("INPUT_CI_FILES", ""))
    parser.add_argument("--ignore-files", default=env("INPUT_IGNORE_FILES", ""))
    args = parser.parse_args()

    if run("git", "rev-parse", "--is-shallow-repository").strip() == "true":
        sys.exit("The repository is a shallow clone. Check it out with fetch-depth: 0.")

    diff_ref = None if args.all else args.diff or diff_base_from_event()
    external = clone_external(args.external.split())
    workspace = colcon_list()

    changed = (
        changed_packages(diff_ref, args.ci_files.split(), args.ignore_files.split())
        if diff_ref else None
    )
    if changed is None:
        selected = set(workspace)
    elif changed:
        selected = colcon_list("--packages-above", *sorted(changed))
    else:
        selected = set()
    selected -= set(args.exclude.split())

    root = split_root(selected)
    folder = {name: str(path) for path, name in package_paths().items()}
    outputs = {
        "skip": "true" if not selected else "false",
        "root-build": " ".join(sorted(root)),
        "root-test": " ".join(sorted(root & selected)),
        "packages": json.dumps([
            {"package": pkg, "path": folder.get(pkg, ""), "repo": external.get(pkg, "")}
            for pkg in sorted(selected - root)
        ]),
    }

    write_summary(diff_ref or "nothing (full build)", changed, selected, root)
    lines = [f"{key}={value}" for key, value in outputs.items()]
    print("\n".join(lines))
    if output := env("GITHUB_OUTPUT"):
        with open(output, "a", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
