package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class DirectionsTest {
    @Test
    public void reginaldAsBuiltTakesPollenInOnPositivePower() {
        Directions built = Directions.reginalds();

        assertFalse(built.reversed(Mechanism.INTAKE));
        assertTrue(built.asBuilt());
        assertEquals("takes_in", Mechanism.INTAKE.way(built.reversed(Mechanism.INTAKE)));
    }

    @Test
    public void reversingTheIntakeMakesPositivePowerPushPollenOut() {
        Directions reversed = Directions.reginalds().with(Mechanism.INTAKE, true);

        assertTrue(reversed.reversed(Mechanism.INTAKE));
        assertFalse(reversed.asBuilt());
        assertNotEquals(Directions.reginalds(), reversed);
        assertEquals(Directions.reginalds(), reversed.with(Mechanism.INTAKE, false));
        assertEquals("intake pushes out on positive power", reversed.toString());
    }

    @Test
    public void aMechanismIsNamedAsTheDescriptionAsksForIt() {
        assertEquals(Mechanism.INTAKE, Valid.value(Mechanism.named("intake")));
        assertEquals(
                "a mechanism is intake, spinner, flywheel_front, flywheel_back, not 'outtake'",
                Mechanism.named("outtake").fold(found -> "found " + found, rule -> rule));
    }

    @Test
    public void reginaldsSpinnerAsBuiltTurnsClockwiseFromAboveOnPositivePower() {
        Directions reversed = Directions.reginalds().with(Mechanism.SPINNER, true);

        assertFalse(Directions.reginalds().reversed(Mechanism.SPINNER));
        assertEquals("turns_clockwise", Mechanism.SPINNER.way(false));
        assertEquals("Positive power turns it counterclockwise from above.", Mechanism.SPINNER.says(true));
        assertEquals("spinner turns counterclockwise on positive power", reversed.toString());
    }

    @Test
    public void aWayIsOneOfTheTwoTheMechanismHas() {
        assertEquals(false, Valid.value(Mechanism.INTAKE.reversedIf("takes_in")));
        assertEquals(true, Valid.value(Mechanism.INTAKE.reversedIf("pushes_out")));
        assertEquals(
                "the intake takes_in or pushes_out on positive power, not 'spins'",
                Mechanism.INTAKE.reversedIf("spins").fold(found -> "found " + found, rule -> rule));
    }

    @Test
    public void everyMechanismSaysWhatPositiveDriveDoesEachWay() {
        assertEquals(
                List.of("takes_in", "pushes_out"), List.of(Mechanism.INTAKE.way(false), Mechanism.INTAKE.way(true)));
        assertEquals("Positive power takes pollen in.", Mechanism.INTAKE.says(false));
        assertEquals("Positive power pushes pollen out.", Mechanism.INTAKE.says(true));
        assertEquals("Intake", Mechanism.INTAKE.label());
    }

    @Test
    public void reginaldsFlywheelsAsBuiltTurnCounterclockwiseSeenFromTheirShaftsOnPositivePower() {
        Directions reversed = Directions.reginalds().with(Mechanism.FLYWHEEL_FRONT, true);

        assertFalse(Directions.reginalds().reversed(Mechanism.FLYWHEEL_FRONT));
        assertFalse(Directions.reginalds().reversed(Mechanism.FLYWHEEL_BACK));
        assertEquals("turns_counterclockwise", Mechanism.FLYWHEEL_FRONT.way(false));
        assertEquals("turns_clockwise", Mechanism.FLYWHEEL_BACK.way(true));
        assertEquals("Front flywheel", Mechanism.FLYWHEEL_FRONT.label());
        assertEquals("Back flywheel", Mechanism.FLYWHEEL_BACK.label());
        assertEquals(
                "Positive power turns its motor counterclockwise, seen from its shaft.",
                Mechanism.FLYWHEEL_BACK.says(false));
        assertEquals("flywheel_front turns clockwise on positive power", reversed.toString());
    }
}
