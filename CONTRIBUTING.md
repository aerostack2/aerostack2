# Contributing to AeroStack2

Thanks for getting involved! The guides on [opening an issue](https://aerostack2.org/docs/develop/issues) and [opening a pull request](https://aerostack2.org/docs/develop/pull-requests) cover the whole process.

## Propose features before building them

New features, new packages and architecture changes start with a [feature request](https://github.com/aerostack2/aerostack2/issues/new?template=FEATURE-REQUEST.yml) or a [discussion](https://github.com/aerostack2/aerostack2/discussions). Agree on the design and where it fits before you write the code. Pull requests for these changes without a prior agreement will be closed.

Bug fixes and small improvements can go straight to a pull request.

## Commit messages

Commits and PR titles follow [Conventional Commits](https://www.conventionalcommits.org), for example `feat(as2_core): add a filterable TF listener`. The scope is optional.

## Local hooks

Install the git hooks once per clone:

```sh
pixi run -e lint lefthook install
```

On every commit this checks the message and runs the ament linters (uncrustify, cpplint, flake8, pep257, xmllint) on the staged files. To lint the whole repository, run `pixi run -e lint lefthook run pre-commit --all-files`.
