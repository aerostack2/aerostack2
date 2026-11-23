# Contributing to Aerostack2

Thanks for getting involved! This page explains how to open an issue, a discussion or a pull request. Step-by-step guides are on the website: [opening an issue](https://aerostack2.org/docs/develop/issues) and [opening a pull request](https://aerostack2.org/docs/develop/pull-requests).

Aerostack2 is maintained by a small team, mostly alongside research work. A few minutes reading this page saves everyone time.

## Understand your change

You must understand the code you submit. If you can't explain what your change does and how it interacts with the rest of the stack, it isn't ready.

AI tools are welcome. Read the [AI Usage Policy](AI_POLICY.md) before using them for a contribution.

## Quick guide

### I want to contribute

Pick an open issue and comment that you're working on it. Issues labelled [`starter`](https://github.com/aerostack2/aerostack2/issues?q=is%3Aissue%20is%3Aopen%20label%3Astarter) are a good first step. If you need help, ask on the issue.

### I found a bug

Search [open and closed issues](https://github.com/aerostack2/aerostack2/issues?q=is%3Aissue) first, it may already be fixed. If it hasn't been reported, open a [bug report](https://github.com/aerostack2/aerostack2/issues/new?template=BUG-REPORT.yml) and fill in the template completely, including the ROS distro, platform and steps to reproduce.

If an open issue already matches your problem, react with 👍 instead of commenting "same here". Comment only when you have new information, such as logs or another way to reproduce it.

### I have an idea for a feature

New features, new packages and architecture changes start with a [feature request](https://github.com/aerostack2/aerostack2/issues/new?template=FEATURE-REQUEST.yml) or an [Ideas discussion](https://github.com/aerostack2/aerostack2/discussions/categories/ideas). Agree on the design and where it fits before you write the code. Pull requests for these changes without a prior agreement will be closed.

### I have a fix or a small improvement

Bug fixes and small improvements can go straight to a pull request. Link the issue it addresses, if there is one.

### I have a question

Ask in [Q&A](https://github.com/aerostack2/aerostack2/discussions/categories/q-a). Please don't open an issue for questions.

## Pull requests

- Keep each pull request to one change. Split unrelated fixes and refactors into separate pull requests.
- Fill in the [pull request template](.github/PULL_REQUEST_TEMPLATE.md), including the distro and platform you tested on.
- Pull requests are not the place to discuss design. Use the feature request or discussion and link your branch there.
- All CI checks must pass before review.

## Commit messages

Commits and PR titles follow [Conventional Commits](https://www.conventionalcommits.org), for example `feat(as2_core): add a filterable TF listener`. The scope is optional.

## Local hooks

Install the git hooks once per clone:

```sh
pixi run -e lint lefthook install
```

On every commit this checks the message and runs the ament linters (uncrustify, cpplint, flake8, pep257, xmllint) on the staged files. To lint the whole repository, run `pixi run -e lint lefthook run pre-commit --all-files`.
