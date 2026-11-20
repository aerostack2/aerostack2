# AGENTS.md

Instructions for AI coding agents working in this repository. The contribution process for humans is in [CONTRIBUTING.md](CONTRIBUTING.md) and the guides at [aerostack2.org](https://aerostack2.org/docs/develop/pull-requests). Everything you produce here also falls under the [AI Usage Policy](AI_POLICY.md): disclose your use in PRs and never report tests you did not run.

## Repository

- A ROS 2 monorepo of colcon packages, targeting Humble (Ubuntu 22.04) and Jazzy (Ubuntu 24.04). Changes must build and pass tests on both.
- `aerostack2/` is the metapackage. The `as2_*` folders are package groups, each package has its own `package.xml`, `CMakeLists.txt` or `setup.py`, and `tests/`.
- Aerial platform drivers (PX4, Crazyflie, Tello, DJI, ...) live in separate `aerostack2/as2_platform_*` repositories. CI clones them, so a change to a shared interface such as `as2_msgs` or `as2_core` must not break them.
- Do not edit generated or vendored code: `ekf_c_code.*` (CasADi), `**/thirdparty/**`, `pnpoly.hpp`, the `indi_control` Gazebo plugin.

## Before you change anything

- New features, packages and architecture changes need an agreed feature request or discussion first. Pull requests without one are closed. Bug fixes and small improvements can go straight to a PR.
- Keep each PR to one concern. Do not mix refactors, formatting or unrelated fixes into a change.
- Read the package and its tests before editing. Follow the conventions already in the file.

## Build and test

```sh
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --ignore-src --from-paths .
colcon build --packages-up-to <package> --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select <package>
colcon test-result --verbose
```

- Build and test only the packages you touched and their dependents (`--packages-above <package>`). A full build is slow.
- C++ tests use gtest, Python tests use pytest, both under the package's `tests/` folder and registered in its `CMakeLists.txt` or `setup.py`.
- A bug fix comes with a test that fails without it. New logic comes with tests.
- `colcon test` also runs the ament linters and copyright check (`ament_lint_auto`). A lint failure is a test failure.

## Code style

- C++17, formatted by `ament_uncrustify` and checked by `ament_cpplint`. Python is checked by `ament_flake8` and `ament_pep257`. XML by `ament_xmllint`.
- Every source file starts with the BSD 3-Clause license header used across the repo. Copy it from a neighbouring file.
- In C++ classes, data members go before methods.
- Comment only a non-obvious reason. Do not restate what the code does.
- Prefer existing helpers in `as2_core` and `as2_python_api` over new ones.
- Launch files are intentionally duplicated across packages. Do not factor them out.

## Git hooks

Install once per clone, with [pixi](https://pixi.sh):

```sh
pixi run -e lint lefthook install
```

On commit, [lefthook](lefthook.yml) runs commitlint on the message and the ament linters on staged files. Lint the whole repository with:

```sh
pixi run -e lint lefthook run pre-commit --all-files
```

Do not bypass the hooks with `--no-verify`. Fix the reported issue instead.

## Commits and pull requests

- Commit messages and PR titles follow [Conventional Commits](https://www.conventionalcommits.org), checked by [commitlint](commitlint.config.mjs). The scope is the package name, for example `fix(as2_core): handle empty frame ids`. Types in use are `feat`, `fix`, `docs`, `test`, `refactor`, `perf`, `build`, `ci`, `chore`. Wrap body lines at 100 characters.
- Commits you write or substantially write end with a `Co-authored-by:` trailer naming the agent, in the form your tool uses (for example `Co-authored-by: Claude <noreply@anthropic.com>`). Keep it when amending, rebasing or squashing.
- PR bodies follow [the template](.github/PULL_REQUEST_TEMPLATE.md), including the ROS distro and aerial platform you tested on, and link the issue they address.
- Breaking changes to messages, parameters or public APIs need `!` in the type (`feat(as2_msgs)!: ...`) and a note in the PR.
- Do not commit `build/`, `install/`, `log/` or local tooling folders.

## CI

- The `ci` workflow builds and tests every package on Humble and Jazzy, plus the external platform repositories. PRs that only change docs skip the build.
- All checks must pass before merge. Code owners in [CODEOWNERS](.github/CODEOWNERS) review changes to their packages.

## Code coverage

- A nightly workflow builds with `--coverage`, runs the tests and uploads C++ (gcovr) and Python (pytest-cov) results to Codecov. Filters are in [gcovr.cfg](gcovr.cfg) and [.coveragerc](.coveragerc).
- To check coverage locally:

  ```sh
  colcon build --packages-select <package> --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=--coverage -DBUILD_TESTING=ON
  colcon test --packages-select <package> --pytest-with-coverage
  gcovr --object-directory build --print-summary
  ```

- New code should not lower the coverage of the package it lands in.

## SonarQube Cloud

- SonarQube Cloud analyses every PR automatically. Settings are in [.sonarcloud.properties](.sonarcloud.properties).
- Fix new bugs, vulnerabilities and code smells it reports on your change. Its duplication warnings on launch files can be ignored.
- Do not silence findings with `NOSONAR` without a reason in the PR.

## Documentation

- User documentation lives at [aerostack2.org](https://aerostack2.org). READMEs link to its guides rather than repeating install or usage steps.
- Document new parameters in the package's config files and note required docs updates in the PR.
- Public C++ APIs use Doxygen comments.
