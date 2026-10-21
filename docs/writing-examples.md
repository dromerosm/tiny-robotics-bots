# Writing an example

Good examples are small, focused and easy to run. Please follow these guidelines.

## Principles

1. **One idea per example.** If you need to explain two unrelated things, make two examples.
2. **Runs out of the box.** It builds with `scripts/build.sh` and needs no extra setup.
3. **Emulator first (StackChan).** If the example can run in esp-emu, make it run there. Simulate missing hardware, and keep the simulation behind a small function that is easy to swap for the real driver.
4. **Safe on the robot.** Never drive servos or motors beyond safe limits. StackChan's tilt servo must stay between 5° and 85°.
5. **Readable over clever.** Short functions, clear names, comments that explain *why*.

## Layout

```
examples/<robot>/<NN-name>/
  README.md            required
  CMakeLists.txt       project(<name>)
  sdkconfig.defaults   must set CONFIG_IDF_TARGET
  smoke.env            required (EMU_CHIP may be empty)
  main/
    CMakeLists.txt     list PRIV_REQUIRES explicitly
    main.c
```

- `<robot>` is `stackchan` or `bala2-fire`.
- `NN` is the next free two-digit number for that robot. Lower numbers are simpler.
- Code shared by several examples goes in `components/<tr_name>/`, written in portable C with host tests in `tests/host/` when possible.

## README

Use the same sections as the existing examples:

1. Title and one-paragraph summary
2. Table: robot, level, emulator support, hardware used
3. What you learn
4. Build
5. Run it (browser, native emulator, robot)
6. Expected output, copied from a real run
7. How it works
8. Try next

## smoke.env

```sh
EMU_CHIP=esp32s3        # empty if esp-emu cannot run it
INJECT_ON="stackchan>"  # optional: wait for this text...
INJECT="help\n"         # ...then type this
EXPECT="text that proves it works"
TIMEOUT=20s             # optional
```

Pick an `EXPECT` that only appears when the example really works, and wait for a prompt that shows the firmware is ready before you inject input.

## Before you open a pull request

```bash
scripts/format.sh                     # needs: pip install clang-format==18.1.8
scripts/test-host.sh
scripts/build.sh examples/<robot>/<NN-name>
scripts/install-esp-emu.sh
scripts/smoke.sh examples/<robot>/<NN-name>
```

Then add the example to the table in the main README.
