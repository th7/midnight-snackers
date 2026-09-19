# Project dev image for the midnight-snackers FTC robot app.
#
# This is a *fragment* — my-agent prepends the `FROM <agent-image>` line at
# build time, so do not add one here. It layers the toolchain needed to compile
# the project (`./gradlew assembleDebug`) onto the base agent image.
#
# The project is an FTC SDK 11.0.0 Android app:
#   - Android Gradle Plugin 8.13.2  -> requires JDK 17+; we use JDK 21, the JDK
#     packaged in Debian 13 (trixie), the agent base image. Gradle 8.13 (from the
#     wrapper) supports JDK 21, and compileOptions still target Java 17 bytecode.
#   - compileSdkVersion 36          -> platform android-36 + build-tools 36.0.0
#   - Gradle itself comes from the committed wrapper (./gradlew), so it is not
#     installed here.
# No NDK is installed: the project has no native (C/C++) sources — the .so files
# ship prebuilt inside the FTC AAR dependencies, and the `abiFilters` in
# build.common.gradle only filter packaging, they don't trigger a native build.

USER root

# JDK (AGP 8.x requires 17+; trixie ships 21) plus tools to fetch the Android SDK.
RUN apt-get update && apt-get install -y --no-install-recommends \
        openjdk-21-jdk-headless \
        curl \
        unzip \
    && rm -rf /var/lib/apt/lists/*

# --- x86-64 runtime so aapt2 works on the arm64 base image ---------------------
# The base image is arm64 (Apple Silicon), but the Android Gradle Plugin ships
# aapt2 only as an x86-64 Linux binary (Google publishes no arm64 build). Docker
# Desktop emulates it via Rosetta, but that needs the x86-64 glibc runtime and
# dynamic loader (/lib64/ld-linux-x86-64.so.2) present — otherwise every AAPT2
# daemon dies with "rosetta error: failed to open elf ..." and all .aar resource
# transforms fail. Adding the amd64 arch and these libs lets Rosetta run AGP's
# own aapt2 transparently (verified end-to-end: assembleDebug produces an APK).
RUN dpkg --add-architecture amd64 \
    && apt-get update && apt-get install -y --no-install-recommends \
        libc6:amd64 \
        libstdc++6:amd64 \
        libgcc-s1:amd64 \
        zlib1g:amd64 \
    && rm -rf /var/lib/apt/lists/*

# Android SDK lives outside /home/agent (that path is masked by the home volume
# at runtime) and outside /workspace (masked by the project bind mount).
ENV ANDROID_SDK_ROOT=/opt/android-sdk
ENV ANDROID_HOME=/opt/android-sdk

# Command-line tools bundle (sdkmanager). Version is the Google build number
# from https://developer.android.com/studio#command-tools .
ARG ANDROID_CMDLINE_TOOLS=13114758

RUN mkdir -p "${ANDROID_SDK_ROOT}/cmdline-tools" \
    && curl -fsSL "https://dl.google.com/android/repository/commandlinetools-linux-${ANDROID_CMDLINE_TOOLS}_latest.zip" \
        -o /tmp/cmdline-tools.zip \
    && unzip -q /tmp/cmdline-tools.zip -d "${ANDROID_SDK_ROOT}/cmdline-tools" \
    && mv "${ANDROID_SDK_ROOT}/cmdline-tools/cmdline-tools" "${ANDROID_SDK_ROOT}/cmdline-tools/latest" \
    && rm /tmp/cmdline-tools.zip

ENV PATH="${PATH}:${ANDROID_SDK_ROOT}/cmdline-tools/latest/bin:${ANDROID_SDK_ROOT}/platform-tools"

# adb here is a client of the host's adb server (the egress firewall blocks the
# robot directly; `egress_allow` in config.yml opens host port 5037). Host must
# run `adb -a start-server && adb connect 192.168.43.1:5555`.
ENV ANDROID_ADB_SERVER_ADDRESS=host.docker.internal

# Accept licenses, then install the exact SDK packages this project compiles
# against. Keep these in sync with compileSdkVersion in build.common.gradle.
RUN yes | sdkmanager --licenses > /dev/null \
    && sdkmanager --install \
        "platform-tools" \
        "platforms;android-36" \
        "build-tools;36.0.0" \
    && chmod -R a+rwX "${ANDROID_SDK_ROOT}"

# --- a browser, for testing the simulator's pages ------------------------------
# The pages are the part of the simulator nothing tests. tools/renderer/check.mjs
# loads field.glb with the three.js the page uses, but it runs in node: no canvas,
# no WebGL, and nothing has ever seen the field drawn. A page that throws on its
# first frame, or puts the field behind the camera, passes every other gate.
#
# Debian's chromium rather than a build fetched from a URL: it is packaged for
# whichever architecture the image is built for, and apt brings the shared
# libraries with it — a long list, and a quiet failure to get wrong. Note that
# amd64 is an added architecture here for Rosetta's sake, so this names no
# architecture and takes the native one.
RUN apt-get update && apt-get install -y --no-install-recommends \
        chromium \
        fonts-liberation \
    && rm -rf /var/lib/apt/lists/*

# Where a test harness finds it. Playwright and Puppeteer each download a browser
# of their own by default, which on linux-arm64 is either missing or a second copy
# of what apt just installed.
ENV CHROME_BIN=/usr/bin/chromium \
    PUPPETEER_SKIP_DOWNLOAD=1 \
    PUPPETEER_EXECUTABLE_PATH=/usr/bin/chromium \
    PLAYWRIGHT_SKIP_BROWSER_DOWNLOAD=1

# Every apt layer above can ship setuid binaries, and chromium's sandbox helper is
# one. The agent drops to uid 1000 at startup and is never meant to climb back, so
# a root-owned entry point reachable from it is only attack surface. Containers
# also run with --security-opt no-new-privileges, but that is a run-time argument
# and the image should not depend on it.
#
# The consequence is that chromium's own sandbox cannot start, so browser tests
# must pass --no-sandbox. See doc/browser-tests.md.
RUN find / -xdev -perm /6000 -type f -exec chmod -s {} +

USER agent

# Run it once here, so a browser that cannot start is a failed build rather than a
# failed test weeks later.
RUN chromium --version \
    && chromium --headless --no-sandbox --disable-gpu --dump-dom about:blank > /dev/null
