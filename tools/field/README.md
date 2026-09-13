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
