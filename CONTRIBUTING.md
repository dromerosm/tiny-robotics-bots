# Contributing

Thanks for your interest in Tiny Robotics examples. Fixes, improvements and new examples are welcome.

## Ways to help

- **Report a problem:** open an issue with the *Bug report* template.
- **Suggest an example:** open an issue with the *Example request* template.
- **Send a fix or an example:** open a pull request.

## Pull requests

1. Fork the repository and create a branch from `main`.
2. Follow the [example guidelines](docs/writing-examples.md).
3. Run the checks locally:
   ```bash
   scripts/format.sh
   scripts/test-host.sh
   scripts/build-all.sh
   scripts/install-esp-emu.sh
   for e in $(scripts/list-examples.sh); do scripts/smoke.sh "$e"; done
   ```
4. Keep pull requests small and focused. Explain what changes and why, and how you tested it (emulator, robot or both).
5. CI must pass before a pull request is merged.

## Code style

- C11, formatted with `clang-format` 18.1.8 and the repository's `.clang-format`.
- ESP-IDF APIs only; no Arduino in this repository.
- Warnings are errors.
- Comments and documentation in English, in plain words.

## Tool versions

ESP-IDF and esp-emu versions are pinned in `tools.env`. If you update them, update the workflows in `.github/workflows/` too; CI checks that they match.

## License

By contributing, you agree that your contributions are released under the [MIT License](LICENSE).
