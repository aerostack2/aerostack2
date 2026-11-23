# AI Usage Policy

Aerostack2 welcomes AI tools, and its maintainers use them. These rules exist because Aerostack2 flies real aircraft and is reviewed by a small team of humans.

## Rules

- **Disclose all AI use.** State the tool (for example Claude Code, Codex, Copilot, Cursor) and how much of the work it did in the pull request description. Commits written by an agent carry its `Co-authored-by:` trailer.
- **Understand every line you submit.** You must be able to explain what your change does and how it interacts with the rest of the stack without the help of an AI tool. If you can't, don't open the pull request.
- **Test what you claim.** The distro and aerial platform in the pull request must be ones you actually ran the change on. Never report simulation or hardware results an AI tool produced or predicted. Code that commands a vehicle must have flown at least in simulation before review.
- **Edit AI-written issues, discussions and reviews.** A human must review and trim the text before posting. Keep the facts, logs and reproduction steps, and drop the filler.
- **The usual process still applies.** Features need an agreed proposal first, and pull requests follow [CONTRIBUTING.md](CONTRIBUTING.md). Agents working in this repository follow [AGENTS.md](AGENTS.md).

Pull requests and issues that don't follow these rules may be closed.

## Why

Every issue and pull request is read by a person. Submitting work you haven't checked moves the cost of checking it onto maintainers, and in robotics an unchecked change can crash a vehicle or hurt someone.

This policy is not against AI. It is about the person driving the tool. Use AI as much as you like, as long as the result is work you understand, have tested and would sign your name to.
