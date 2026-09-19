# Browser tests

The simulator's pages are the part of it nothing tests. `tools/renderer/check.mjs`
loads `field.glb` with the three.js the page uses, which says the model is
sound and the library accepts it, but it runs in node: there is no canvas, no
WebGL, and nothing has ever seen the field drawn. A page that throws on its
first frame, or draws the field behind the camera, or draws nothing at all
because a shader failed to compile, passes every gate we have.

A browser is what closes that. This says what the image needs for one.

## The browser

`.my-agent/Dockerfile` installs Debian's `chromium` and `fonts-liberation`, and
runs the browser once in the layer that installed it so an image that cannot
start one is a failed build rather than a failed test weeks later. Debian's
package rather than a build fetched from a URL: it is packaged for whichever
architecture the image is built for, and apt brings the shared libraries with
it, which is a long list and a quiet failure to get wrong.

`CHROME_BIN` and `PUPPETEER_EXECUTABLE_PATH` point at it, and the two download
switches are off, so a harness uses that browser rather than fetching a second
copy — which on linux-arm64 is either missing or a duplicate.

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

## The test

`tools/browser/check.mjs`, driven by Playwright, run by `python3 tools/run_tests.py` with
everything else. It serves the simulator's resource directory over a throwaway HTTP
server -- the page at `/field`, its assets under `/assets`, exactly as the bench lays
them out -- opens the page, and asks what nothing else can ask:

- the page threw nothing and no request it made failed;
- the model reached the page, with its parts and its triangles;
- and a frame actually rasterised some of them.

That last one is the point. `renderer.info.render.triangles` after a frame is how a field
that loaded and was not drawn is told from one that was: a camera pointed away, or a
frustum that culls everything, leaves a page that has fetched every byte it asked for and
shows the background colour. The page exposes `window.fieldPage` for this and nothing
else.

A failing run writes `tools/browser/field-as-drawn.png`, which CI keeps with the test
results, because the first question about a page that did not draw is what it did draw.

## Two browsers, and why

In the image, Chromium comes from apt and `CHROME_BIN` points at it; Playwright's own
download is switched off, because there is no linux-arm64 build it would rather use.

In CI, on `ubuntu-latest`, `npx playwright install --with-deps chromium` fetches
Playwright's build and the shared libraries it needs, and `CHROME_BIN` is unset so
Playwright uses its own — asked for as `channel: 'chromium'`, the full browser.
Playwright's default is `chrome-headless-shell`, which is the faster build and the one
without the graphics stack WebGL needs. A page that draws does not draw in it, and the
way that failure arrives is a timeout waiting for a page that threw while making its
renderer, which says nothing about WebGL at all.

So the command is the same in both places and the browser is not quite. Both are
Chromium, both render through SwiftShader, and the checks are about whether a page draws
rather than about exact pixels -- but it is a difference, and if a page test ever
disagrees between the two, this is the first place to look.

## What a test can then say that nothing else can

- The page loads with no uncaught error and no failed request.
- `THREE.WebGLRenderer` starts, which is where a missing extension or a shader
  that will not compile shows up.
- The scene has the parts, and the camera is looking at them: a render that
  produces only the background colour is a field off screen, and no other gate
  we have can tell that from a field drawn correctly.
- The controls orbit, and the page survives a resize.

