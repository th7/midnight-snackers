# Browser tests

The simulator's pages are the part of it nothing tests. `tools/renderer/check.mjs`
loads `field.glb` with the three.js the page uses, which says the model is
sound and the library accepts it, but it runs in node: there is no canvas, no
WebGL, and nothing has ever seen the field drawn. A page that throws on its
first frame, or draws the field behind the camera, or draws nothing at all
because a shader failed to compile, passes every gate we have.

A browser is what closes that. This says what the image needs for one.

## What to add to `.my-agent/Dockerfile`

This is a fragment to merge into the existing one, not a replacement — end the
file as `USER agent` as it already does.

```dockerfile
# --- a browser, for the page tests ------------------------------------------------
# Debian's chromium rather than a build fetched from a URL: it is packaged for
# whichever architecture the image is being built for, and apt pulls the shared
# libraries it needs, which is a long list and a quiet failure to get wrong.
USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        chromium \
        fonts-liberation \
    && rm -rf /var/lib/apt/lists/*

# Where a test harness finds it. Playwright and Puppeteer both download a browser
# of their own by default, which for linux-arm64 is either missing or a second
# copy of what apt just installed.
ENV CHROME_BIN=/usr/bin/chromium \
    PUPPETEER_SKIP_DOWNLOAD=1 \
    PUPPETEER_EXECUTABLE_PATH=/usr/bin/chromium \
    PLAYWRIGHT_SKIP_BROWSER_DOWNLOAD=1

# apt reintroduces setuid bits, chromium's sandbox helper among them.
RUN find / -xdev -perm /6000 -type f -exec chmod -s {} +

USER agent
# Run it once in the layer that installed it, so a browser that cannot start is a
# failed build rather than a failed test weeks later.
RUN chromium --version \
    && chromium --headless --no-sandbox --disable-gpu \
        --dump-dom about:blank > /dev/null
```

## Why the flags

**`--no-sandbox` is not optional here.** Chromium sandboxes itself with a setuid
helper, the image strips every setuid bit on purpose, and containers run with
`--security-opt no-new-privileges`. The sandbox cannot start, so a test that
omits the flag hangs or dies with a message about the sandbox rather than about
the page. The browser is only ever pointed at our own bench, so the trade is
one we can make deliberately — but it is a trade, and it is why this is written
down rather than discovered.

**`--disable-dev-shm-usage`** belongs in the test harness, not the image: a
container's `/dev/shm` is 64 MB by default and Chromium will run out of it part
way through a page and crash in a way that reads as a flaky test.

**WebGL needs SwiftShader.** There is no GPU, so Chromium falls back to its
software rasteriser. Newer Chromium refuses that fallback unless asked:

```
--enable-unsafe-swiftshader --use-gl=angle --use-angle=swiftshader
```

Software rendering is slow, and it is also the only way these tests come out the
same on two machines. A GPU would make them faster and make the pixels depend on
a driver — which for a test that looks at pixels is the difference between a
verdict and a coin toss.

## What a test can then say that nothing else can

- The page loads with no uncaught error and no failed request.
- `THREE.WebGLRenderer` starts, which is where a missing extension or a shader
  that will not compile shows up.
- The scene has the parts, and the camera is looking at them: a render that
  produces only the background colour is a field off screen, and no other gate
  we have can tell that from a field drawn correctly.
- The controls orbit, and the page survives a resize.

## Not settled

CI runs on `ubuntu-latest`, not in this image, so a browser test in
`.github/workflows/tests.yml` depends on what that runner has rather than on
this fragment. Decide that before writing the first test, or the local command
and the CI command stop being the same command.
