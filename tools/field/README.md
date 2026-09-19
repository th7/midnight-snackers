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

`onshape.py` is the client. Most of that document is public and needs no
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

    python3 tools/field/onshape.py --check

`--check` makes one call and says whether Onshape answered it. The unit
tests pin the signing algorithm so it cannot drift silently, but only
Onshape can say a signature is right, which is what that call is for. A
geometry call nothing authenticated raises, naming both things it could be,
so a run that regenerates nothing cannot look like a run that worked.

`gltf.py` reads what the export answers with -- a `.glb`, or the JSON with
its buffer inline -- into a flat list of parts, each with its node's name,
the names of the nodes above it, its triangles already placed in the
assembly's frame, and its colour. It reads only what the pipeline needs,
and raises on anything else rather than skipping it: a part silently
dropped, or one read at the wrong stride, would reach the simulator as
geometry that looks plausible and is wrong.

`assets.py` fetches the document's images -- the panel artwork and the four
goal April Tags -- which is the part of the field STEP cannot carry, and
needs no keys:

    python3 tools/field/assets.py --fetch

Nothing is written until every image has arrived and been checked, so a
rename upstream stops the run rather than quietly fetching less than it
used to.

## Two models, deliberately

`field.json` is the field the simulator **collides**: each part a convex
shape, built from FIRST's published STEP by `step_to_field.py`.

`field.glb` is the field as it **looks**: the Onshape assembly's real
tessellation, fillets and all, built by `field_glb.py`.

    python3 tools/field/field_glb.py                    # fetch and build
    python3 tools/field/field_glb.py --export f.gltf    # from an export in hand

They are not the same model and are not meant to be. A physics engine wants
convex shapes and a renderer wants the detail, and one model cannot be good
at both. What they must agree about is where the field is, which the build
checks before it writes anything: blue's parts at negative y and red's at
positive, which a quarter turn the wrong way swaps while leaving a field
that otherwise looks entirely reasonable, and a reach that matches the
walls, which a frame read in metres does not. A check that has too little to
go on says so rather than passing.

The export is a million and a half triangles, most of them spent on fillets
nobody can see at the size of a field -- the two goal ribs alone are nearly
half of them. Points are snapped to a twentieth of an inch and the triangles
that collapse are dropped, which leaves about a sixth of them, three
megabytes, and moves nothing further than a twenty-third of an inch.

We take whatever the document holds at the time we refresh the assets:
nothing here pins an Onshape version, and small changes between refreshes
are expected.

## Running the tests

The scripts under `tools/` are tested by the same command locally and in CI:

    python3 tools/run_tests.py
