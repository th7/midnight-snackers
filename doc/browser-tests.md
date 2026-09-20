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

## What a frame costs

`tools/browser/cost.mjs` is the same browser put to a different question: not
whether the page draws, but what drawing it costs.

    node tools/browser/cost.mjs                 # measure, and check the budget
    node tools/browser/cost.mjs --regenerate    # record a new budget

The probe is `framecost.js`, served with the page's other assets, and the page
itself runs it: **<http://localhost:21986/sim/field?run=1&cost>** measures and
prints the reading into the HUD. That is how a frame is measured on a tablet,
where no harness runs -- open the bench's address on the device and read it.

Three things make the number mean something.

**It renders as fast as it can, not once a frame.** A page held to the
display's refresh reports 60 fps whether a frame takes 2 ms or 16, so the probe
renders in a tight loop instead, outside `requestAnimationFrame`.

**It waits for the GPU.** `render()` returns when the commands are queued, not
when they are drawn, so a block of renders is followed by a one-pixel
`readPixels`, which cannot answer until the queue has drained. The reading
carries both: the time to *submit* a frame, which is the CPU's share, and the
time for one to *finish*. A frame that is cheap to submit and dear to finish is
GPU-bound; the other way round is draw calls.

**It renders enough of them to out-measure the clock.** `performance.now()` is
deliberately coarse, so the probe doubles the number of renders in a block
until the block takes 50 ms, and divides. On a machine where even the cap is
too quick, it reports that it could not judge rather than a number that is the
clock's resolution. A measurement nobody could take must not read like one that
was.

**Draw calls are counted, not asked for.** `renderer.info.render.calls` counts
the colour pass only: five shadow-casting meshes report five calls whether the
shadow pass runs or not, though the GPU drew them ten times. So the probe wraps
the context's `drawElements` and `drawArrays` for one frame and counts what
actually happens. The shadow pass turned out to be nearly half of them, which
no number three.js reports would have said.

## Batching, and what it did and did not buy

The field model has 305 parts and 11 materials, and it was drawn as 305
meshes: a draw call for each, and another for each shadow caster. `fieldscene.js`
now merges the geometry of every part that shares a material into one mesh,
which took a frame from **620 draws to 164 with shadows, and 338 to 89
without**, and the geometries on the GPU from 370 to 89. Not everything can
merge: the goal tags are read back by name to work out where a tag hangs, and
anything that has to move or be hidden by itself -- each hive's parts, the
tape, the game pieces -- is merged only within the group that moves it. What a
merged mesh was made of is kept in its `userData.from`, so the scene can still
be asked whether the perimeter is in it.

The triangles are unchanged, and a merge that lost any fails the load rather
than drawing a field that is quietly missing parts.

**It bought nothing measurable here, and that is the expected result.** Frame
time on SwiftShader did not move: 161 ms before and 165 ms after with shadows,
which is inside the noise. Software rasterising 266,539 triangles is what this
machine spends a frame on, and issuing the draw calls was never the cost. What
did move is the half of the reading that measures the CPU: submitting a frame
went from 1.5 ms to 0.7 ms with shadows, and 0.9 ms to 0.5 ms without.

That is the whole case for the change, and it is a case about a machine we
cannot test on. A tablet's GPU rasterises this scene without noticing; what
costs it is per-draw driver work, which is the number that fell. So the change
is justified by the draw count and the submit time, not by a frame time -- and
the frame time on a real device is still an open question that only a device
can close.

One trade-off comes with it: a merged mesh is culled as a whole, so a camera
that used to cull individual parts may now draw them. At the field's size, with
the whole field usually in view, nothing measurable changed.

## The budget

`tools/browser/scene-budget.json` records the counts that do not vary with the
machine -- draws, colour-pass calls and triangles -- and the check fails when
they move. Timings are printed and never asserted: they are a property of
whatever ran them, and SwiftShader's are a hundred times a real GPU's.

It is a change detector, not a judgement, and the same deal as a golden trace:
a missing budget fails rather than passing for want of anything to compare, and
a regenerating run fails too, because a run that wrote the answer down has not
checked it. An intended change is read and then regenerated.

## What a test can then say that nothing else can

- The page loads with no uncaught error and no failed request.
- `THREE.WebGLRenderer` starts, which is where a missing extension or a shader
  that will not compile shows up.
- The scene has the parts, and the camera is looking at them: a render that
  produces only the background colour is a field off screen, and no other gate
  we have can tell that from a field drawn correctly.
- The controls orbit, and the page survives a resize.

