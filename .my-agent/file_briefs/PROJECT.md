## Working with this project (midnight-snackers, FTC)

FTC robot app — Android / Gradle / Kotlin, FTC SDK 11.0.0. Compile with
`./gradlew assembleDebug`.

### SDK location
- `local.properties` is gitignored and intentionally leaves `sdk.dir` unset in the
  container, so the build falls back to `ANDROID_HOME=/opt/android-sdk`. Do not add
  a `sdk.dir` line in the container.

### Build environment (already handled in `.my-agent/Dockerfile`)
- JDK 21, Android SDK platform-36 + build-tools 36.0.0.
- The base image is arm64 but Google ships aapt2 only for x86-64; it runs under
  Rosetta thanks to the amd64 glibc the Dockerfile installs. If a build fails on
  aapt2 or an "architecture / rosetta" error, read the Dockerfile comments before
  changing anything.

### Deploying to the robot (adb)
- Container `adb` is a client of the **host's** adb server (`ANDROID_ADB_SERVER_ADDRESS`
  is set in the Dockerfile). Do not `adb connect` to the hub from the container.
- If `adb devices` hangs or shows no device, ask the user to run on the host:
  `adb kill-server && adb -a start-server && adb connect 192.168.43.1:5555`

### Git
- Personal per-checkout ignores (`.claude/`, `.my-agent/`) live in `.gitignore.local`,
  wired via `git config core.excludesFile` — local to this checkout, not synced.
