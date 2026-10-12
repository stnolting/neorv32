## Contributing

Contributions are very welcome! If you have any questions, bug reports, ideas
or if you want to give some kind of feedback, feel free to open a
[new issue](https://github.com/stnolting/neorv32/issues/new/choose)
or start a new [discussion](https://github.com/stnolting/neorv32/discussions).

Please make sure your changes are clear, well-documented, and focused.
For larger/breaking changes, please discuss them first via an issue.

### Code of Conduct

Note that we have a [Code of Conduct](https://github.com/stnolting/neorv32/blob/main/CODE_OF_CONDUCT.md).
By participating and contributing to this project, you agree to uphold our Code of Conduct.

### Documentation

If your changes affect behavior, configuration, or usage of the project, please update the
relevant documentation (e.g. files under `docs/`) accordingly. Keeping code and documentation
in sync makes reviews easier and helps other contributors and users.

### Testing

If applicable, please verify that your changes do not break existing functionality by running
the available testbenches / simulations before opening a pull request. If you add new
functionality, consider adding or extending tests where reasonable.

### Contributing Code

Here is a simple guide line if you'd like to contribute code modifications to this project:

1. [Fork](https://github.com/stnolting/neorv32/fork) this repository and clone the fork: `git clone https://github.com/stnolting/neorv32.git`
2. In your local copy, create a feature branch: `git checkout -b awesome_new_feature_branch`
3. Create a new _remote_ for the upstream repo: `git remote add upstream https://github.com/stnolting/neorv32`
4. Commit your modifications: `git commit -m "Awesome new feature!"`
5. Push to the branch: `git push origin awesome_new_feature_branch`
6. Create a new [pull request](https://github.com/stnolting/neorv32/pulls); please make sure that your feature branch is up-to-date
with the project's `main` branch; we will review your request as soon as possible. Smaller, focused pull requests are easier to review
and merge than large, mixed-purpose ones; consider splitting unrelated changes into separate pull requests.
7. If you like, discuss / show-case your work on the project's [discussion board](https://github.com/stnolting/neorv32/discussions).

### Coding Style

Please try to follow the formatting rules defined in the project's [`.editorconfig`](.editorconfig)
file (e.g. indentation, line endings, trailing whitespace) whenever possible. Most editors and IDEs
support [EditorConfig](https://editorconfig.org/) natively or via a plugin, so your changes should
automatically match the project's style.

### Design Philosophy

A key design goal of NEORV32 is to keep the processor core small and resource-efficient.
while providing maximal RISC-V functionality. Hardware changes that increase the core's size
should therefore only be introduced if at least one of the following applies:

- The added logic can be disabled by configuration, so it does not consume resources when not required (for example, optional ISA extensions).
- The change provides a significant increase in functionality or capability that justifies the additional hardware resources.
- The change is required to fix a functional, reliability, or security issue.

In general, minimizing logic utilization and resource footprint has higher priority than
maximizing clock frequency. Performance-oriented changes should not substantially increase
core size unless their benefit clearly justifies the trade-off.

### Use of AI Tools

AI tools may be used when contributing to this project (e.g. to refine wording,
polish documentation, or assist with code suggestions). However, such tools should primarily
be used to refine and improve existing ideas - not to generate new ideas, designs, or
decisions on your behalf.

All AI-assisted content must be carefully reviewed, verified, and understood by the
contributing author before being submitted.

### License of Contributions

By submitting a contribution to this project (including but not limited to pull
requests, patches, documentation, or other materials), you agree that:

- You are the original author of the contribution, or you have the right to submit it under the terms below.
- You grant this project a perpetual, worldwide, non-exclusive, royalty-free license to use, modify, distribute, and sublicense your contribution.
- Your contribution will be licensed under the same [license](LICENSE) as the project.

If you do not agree to these terms, please do not submit a contribution.
