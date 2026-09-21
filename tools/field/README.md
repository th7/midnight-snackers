# The field model

`step_to_field.py` turns the season's field CAD into the simulator's low-poly
field model, `TeamCode/src/test/resources/org/firstinspires/ftc/teamcode/sim/field.json`,
which `SimField` reads and the replay page draws.

FIRST publishes the CAD as a STEP assembly. This season's is
<https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad-step>
("BIOBUZZ_Full Field.20260912.step", 35 MB, not kept in the repository). To
regenerate the model after FIRST revises the CAD:

    curl -L -o field.step https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad-step
    python3 tools/field/step_to_field.py field.step
    ./gradlew :TeamCode:testDebugUnitTest

The script needs only Python 3. What it keeps, drops and simplifies is
described at the top of the script; `SimFieldTest` checks the result.

## Onshape

FIRST also publishes the field as an Onshape document, "BIOBUZZ™ Playing
Field", which holds the same STEP import plus what STEP cannot carry: the
panel artwork and the four goal April Tag images, as blobs.

<https://cad.onshape.com/documents/a355e772e3d24813de7852ee/w/f106353168f1f92100b81259/e/95d1e1e442b4138cccaf2d73>

`Onshape` is the client. Most of that document is public and needs no
credentials -- its metadata, its element listing, its BOM and its blobs.
Only geometry is authenticated, and there are two ways that happens.

Under my-agent, an egress proxy signs `cad.onshape.com` on the way out: the
container holds no keys and needs none. Because the signature covers the
path and query of the request the proxy finally sends, one computed in the
container would not match, so requests go out bare and `Authorization`,
`Date` and `On-Nonce` are the proxy's to set.

Away from the proxy -- a teammate's laptop -- the key pair comes from
<https://dev-portal.onshape.com>, in the environment and never in the
repository:

    export ONSHAPE_ACCESS_KEY=...
    export ONSHAPE_SECRET_KEY=...

Either way:

**Refresh assets** on the admin page makes the call and says whether Onshape
answered it. `OnshapeTest` pins the signing algorithm so it cannot drift
silently, but only Onshape can say a signature is right, which is what that
button is for. A geometry call nothing authenticated is refused naming both
things it could be, so a refresh that fetched nothing cannot look like one
that worked.

`Gltf` reads what the export answers with -- a `.glb`, or the JSON with
its buffer inline -- into a flat list of parts, each with its node's name,
the names of the nodes above it, its triangles already placed in the
assembly's frame, and its colour. It reads only what the pipeline needs,
and raises on anything else rather than skipping it: a part silently
dropped, or one read at the wrong stride, would reach the simulator as
geometry that looks plausible and is wrong.

`FieldAssets` fetches the document's images -- the panel artwork and the four
goal April Tags -- which is the part of the field STEP cannot carry, and
needs no keys.

Nothing is written until every image has arrived and been checked, so a
rename upstream stops the run rather than quietly fetching less than it
used to.

## Two models, deliberately

`field.json` is the field the simulator **collides**: each part a convex
shape, built from FIRST's published STEP by `step_to_field.py`.

`field.glb` is the field as it **looks**: the Onshape assembly's real
tessellation, fillets and all, built by the coding server rather than kept in
the repository. *Its* tessellation, and not one we choose -- see below. Press **Refresh assets** on the admin page, or start the server
without having fetched them and it fetches in the background. Classes:
`Onshape`, `Gltf`, `FieldGlb`, `FieldAssets`.

The admin page chooses the **resolution**, in three steps -- both which one to
build, and which one the pages draw when they ask for none:

| | parts | triangles | bytes |
|---|---|---|---|
| low | 305 | 316,000 | 4.0 MB |
| medium | 305 | 1,020,000 | 27.5 MB |
| high | 1,029 | 1,474,500 | 41.7 MB |

Low is the field as it plays: snapped to a twentieth of an inch, a colour to a
part, and without the hardware — two thirds of the assembly is screws, nuts,
rivets, cable ties, the soft tiles and what is under them, and nothing draws a
washer once the field is up. Medium is those same parts at the CAD's own points
and in the CAD's own materials. High adds the hardware back.

A page draws at **high** unless it asks for less with `?resolution=low` or
`medium` — or unless somebody picks another from the selector under the field,
which is the same choice made while looking at it. Which one it draws when it
asks for none is **Pages draw** on the admin page, kept with the rest of the
server's state and read by every page as one served line, `field-default.js`;
high is what a server says until an admin sets another, and what every server
with no admin says. A page that asks for a resolution nobody built draws the
low one and says so rather than failing. `all` builds the three. They sit side by side, so
changing resolution is a query string rather than another fetch. None of it
costs many more draw calls, because the scene is batched by material rather
than by part.

### What comes across, and what is not there to come

Points always; **normals** and whole **materials** at medium and high. Normals
matter as much as points: without them a page computes its own, averaging across
every face a vertex touches, which rounds off every edge the CAD meant to be
sharp. They also roughly double a model, which is why the lowest goes without
them and keeps a colour to a part, at 4.0 MB. Resolution decides two things:
which parts are in the model, and how much of their surface comes with them.

**There are no UVs and no textures in the export.** Not dropped: absent. The
assembly's glTF carries `POSITION` and `NORMAL` and nothing else per vertex,
and no `images`, `textures` or `samplers` at all. So the panel and tag artwork
cannot be applied from the CAD's own mapping, and the goal tags are drawn on
quads the page builds for itself. A material that named a texture would be
written as a dangling reference, so the writer refuses one instead.

### How fine the tessellation is, and why we do not choose

The export is `GET /api/v10/assemblies/d/{did}/w/{wid}/e/{eid}/gltf`, and it
takes **no tessellation parameters**. That is measured, not assumed: the same
request with `angleTolerance` and `chordTolerance` at 0.4 and 0.01 (very
coarse), at 0.02 and 0.00002 (very fine), with `maxFacetWidth=0.05`, and with
`precomputedLevelOfDetail=coarse` all came back byte-for-byte identical, at
11,258,655 bytes. Onshape's own OpenAPI spec agrees: that path is not in it.
The documented GET that *does* take `angleTolerance`, `chordTolerance`,
`maxFacetWidth` and `precomputedLevelOfDetail` is the **part studio** one,
`/partstudios/.../gltf`, and it exports a part studio rather than an assembly.

(An earlier attempt used `angularTolerance`, which is the wrong name for that
endpoint family and would have been ignored whatever the endpoint did. The
graphics endpoints spell it `angleTolerance`; the translation ones spell it
`angularTolerance`. Both were tried.)

So the only documented way to ask an *assembly* for a different tessellation is
the translation API: `POST /assemblies/d/{did}/{wv}/{wvid}/e/{eid}/export/gltf`
with a `BTBGltfExportParams` body carrying `meshParams`
(`angularTolerance`, `distanceTolerance`, `maximumChordLength`, `resolution`
of `FINE|MEDIUM|COARSE`). That is asynchronous -- it makes a translation, which
is then downloaded -- so taking it up means a different shape of fetch, not a
different query string. Nobody has needed it yet: what high resolution was missing
was parts, not points.

Until somebody does that, **"the CAD's own tessellation" means "whatever the
default export gave us"**, and nothing here should claim more.

**Downloading and building are separate asks**, because they cost such
different things. *Download* fetches the assembly and the textures from Onshape
— tens of seconds, eleven megabytes — and keeps the export at `export.gltf` in
the state directory. *Build* makes the models from that export with no network
at all, in under a second, so changing the resolution or rebuilding after a change
to the pipeline costs nothing but the arithmetic. *Refresh assets* is the two in
one, and is what a server with nothing fetched runs at startup — nothing
fetched meaning the field its pages draw, so a server set to draw the cheap
model does not fetch the dear one to sit unlooked at. A build with
nothing downloaded is refused and says to download first, rather than quietly
reaching for the network: `FieldAssets.build` is handed no Onshape, so it could
not fetch if it wanted to.

The one in the repository is a **stand-in**: the model the tests draw, frozen,
so the browser checks and the scene budget come out the same on any machine
with no network. A server that has fetched draws what it fetched; one that has
not draws the stand-in and says so on the admin page.

They are not the same model and are not meant to be. A physics engine wants
convex shapes and a renderer wants the detail, and one model cannot be good
at both. What they must agree about is where the field is, which the build
checks before it writes anything: blue's parts at negative y and red's at
positive, which a quarter turn the wrong way swaps while leaving a field
that otherwise looks entirely reasonable, and a reach that matches the
walls, which a frame read in metres does not. A check that has too little to
go on says so rather than passing.

The export is 1,474,544 triangles over 1,029 parts, most of them spent on
fillets nobody can see at the size of a field -- the eight goal ribs alone
are 413,072 of them, 28% of the export, at 51,634 triangles each. The name
rules drop 724 parts and leave 1,020,128 triangles; points are then snapped
to a twentieth of an inch and the triangles that collapse are dropped, which
leaves 315,988 -- 21% of the export, 305 parts, four megabytes -- and moves
nothing further than a twenty-third of an inch.

The perimeter is kept here and dropped by the collision model, which models
the walls itself: a field drawn without it is a floor with things standing
on it. Its parts have to be named to be kept, because they are named for
what the hardware rule drops -- the rail is "FTC Rail with Rivet Holes".

We take whatever the document holds at the time we refresh the assets:
nothing here pins an Onshape version, and small changes between refreshes
are expected.

## The renderer

The bench draws `field.glb` with three.js, vendored under the simulator's
resources (`vendor/`, MIT, in `doc/legal`) so the page loads nothing from the
network -- the robot's wifi has none.

**One page draws the field**: the live view, at `/runs/<id>/`, which under the
coding server means **<http://localhost:21986/sim/runs/7/>**. It plays a run
back -- the robot where the simulator had it, the game pieces where they had
rolled to, each hive leaning the way the run left it -- and it takes what is
asked of it in the query string:

    ?view=camera   what the robot's webcam would have seen
    ?view=flat     the flat drawing of what the simulator collides
    ?cost          what a frame costs to draw (the box under the field)
    ?resolution=   low, medium or high, for another than the one it draws by default

The dashboard passes these through, so `#simulate?resolution=low` reaches the
view it embeds. The geometry is all in `field.glb`; what a tick carries is
where things had moved to. What the geometry cannot say -- which pieces move,
where each hive hangs, how big the robot is -- the page already carries, from
the collision model.

`fieldscene.js`, served beside it, is the one renderer of that model: it
builds the scene -- the glb, the lights, the floor, the balls, the hives and
the robot. `webcam.js` adds the goal tags and the lens, `framecost.js`
measures. The page is `replay.html`, which keeps its own flat 2D drawing of
the collision model for the replay file it writes, for `?view=flat`, and for
when the assets cannot be fetched.

`tools/renderer/check.mjs` loads the model with that same three.js, in node.
The writer is already checked by reading back what it writes, which says the
two agree with each other; this says the library the browser hands it to
accepts it, which only the other side can say. It needs node, and a missing
node fails the run rather than skipping the check.

## The camera's view

`/sim/runs/<id>/?view=camera` shows what the robot's webcam would have seen of
the goal tags: the tags where they are, in the perspective the lens gives, and
nothing else. The field is hidden rather than absent, so a tag still leans with
the hive it hangs on as the run plays.

    node tools/browser/frames.mjs --bench http://localhost:21986/sim --run 3 --into frames/

writes one PNG per loop at the camera's own resolution. The coding server wants
an approved session, which `--cookie` carries.

Two things in it are assumed and must be measured before a pose read off these
frames means anything. Both are written at the top of `webcam.js`:

* **The lens.** `teamwebcamcalibrations.xml` is still the SDK's stock file --
  every `<Camera>` block in it is commented out -- so the focal length and
  principal point are a plausible 640x480 webcam and not this team's. A
  detector's range and bearing scale directly with them.
* **The tag's printed size.** The CAD gives a plate 5 in by 17 in; a 36h11
  tag's black square is smaller than the plate, and the season manual is what
  says by how much. The artwork is fitted to the plate keeping its own 3:1
  shape, which is a guess at the printing rather than a measurement of the tag.

Where the tags *are* is not assumed: that comes from the CAD, through the hive
they hang on, and moves as the hive leans.

## Running the tests

The scripts under `tools/` are tested by the same command locally and in CI:

    python3 tools/run_tests.py
