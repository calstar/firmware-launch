# firmware-launch
For firmware used on boards during launch.

# How to build
* Run `make build board=$BOARD` to compile flatbuffer schemas, build the `.bin` file, and copy it to `out/$BOARD.bin`.
    * `$BOARD` must be one of: `bb` (Black Box), `fc` (Flight Computer), `gs` (Ground Station), `tpc` (Telemetry/Power Control).
* Run `make build-all` to simply run `make build board=$BOARD` for all 4 boards sequentially.

# How to flash
* Run `make flash board=$BOARD` if `out/$BOARD.bin` exists to use the `st-flash` command.

# Relevant Folders
* The four boards (`bb`, `fc`, `gs`, `tpc`) each have folders. Simply add code to the appropriate folder.
* The flatbuffer schemas are under `general/*.fbs` (more details below).
* `flatbuffers/` contains the necessary flatbuffer includes and should not be modified.
* After a build, `out/` will contain the output binary.

# Flatbuffers
* The schemas are under `general/*.fbs`. Edit them and the next time you run a `make build board=$BOARD` command they will be recompiled by the `flatc` executable in `general/`.
