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
Only geometry is authenticated, so exporting the assembly needs a key pair
from <https://dev-portal.onshape.com>, in the environment and never in the
repository:

    export ONSHAPE_ACCESS_KEY=...
    export ONSHAPE_SECRET_KEY=...
    python3 tools/field/onshape.py --check

`--check` makes one signed call and says whether Onshape accepted it. The
unit tests pin the signing algorithm so it cannot drift silently, but only
Onshape can say the signature is right, which is what that call is for. A
call that needs keys and has none raises rather than carrying on, so a run
that regenerates nothing cannot look like a run that worked.

`gltf.py` reads what the export answers with -- a `.glb`, or the JSON with
its buffer inline -- into a flat list of parts, each with its node's name,
the names of the nodes above it, its triangles already placed in the
assembly's frame, and its colour. It reads only what the pipeline needs,
and raises on anything else rather than skipping it: a part silently
dropped, or one read at the wrong stride, would reach the simulator as
geometry that looks plausible and is wrong.

We take whatever the document holds at the time we refresh the assets:
nothing here pins an Onshape version, and small changes between refreshes
are expected.

## Running the tests

The scripts under `tools/` are tested by the same command locally and in CI:

    python3 tools/run_tests.py
