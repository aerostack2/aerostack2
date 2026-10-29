# Contributing to AeroStack2

Thanks for getting involved! Information on contributing can be found at [https://aerostack2.github.io/contributing](https://aerostack2.github.io/contributing)

## Commit messages

Commits and PR titles follow [Conventional Commits](https://www.conventionalcommits.org), for example `feat(as2_core): add a filterable TF listener`. The scope is optional.

## Local hooks

Install the git hooks once per clone:

```sh
pixi run -e lint lefthook install
```

On every commit this checks the message and runs the ament linters (uncrustify, cpplint, flake8, pep257, xmllint) on the staged files. To lint the whole repository, run `pixi run -e lint lefthook run pre-commit --all-files`.
