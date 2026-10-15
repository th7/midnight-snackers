package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import org.junit.Test;

public class ConstantsTest {
    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    private static Map<Constant, Double> set(Object... pairs) {
        Map<Constant, Double> set = new EnumMap<>(Constant.class);
        for (int i = 0; i < pairs.length; i += 2) {
            set.put((Constant) pairs[i], ((Number) pairs[i + 1]).doubleValue());
        }
        return set;
    }

    private static Checked<Constants> of(Object... pairs) {
        return Constants.of(set(pairs));
    }

    @Test
    public void everyConstantStartsAtTheValueTheSimulatorWasBuiltWith() {
        Constants built = Constants.defaults();

        assertEquals(Noise.MOTOR_SPREAD, built.value(Constant.MOTOR_SPREAD), 0);
        assertEquals(Noise.FLATTEST_VOLTS, built.value(Constant.FLATTEST_VOLTS), 0);
        assertEquals(Noise.FRESHEST_VOLTS, built.value(Constant.FRESHEST_VOLTS), 0);
        assertEquals(Noise.SAG_VOLTS_PER_POWER, built.value(Constant.SAG_VOLTS_PER_POWER), 0);
        assertEquals(Noise.DRAIN_VOLTS_PER_SECOND, built.value(Constant.DRAIN_VOLTS_PER_SECOND), 0);
        assertEquals(Noise.LEAST_TRACTION_G, built.value(Constant.LEAST_TRACTION_G), 0);
        assertEquals(Noise.MOST_TRACTION_G, built.value(Constant.MOST_TRACTION_G), 0);
        assertEquals(Noise.SET_DOWN_INCHES, built.value(Constant.SET_DOWN_INCHES), 0);
        assertEquals(Noise.SET_DOWN_RADIANS, Math.toRadians(built.value(Constant.SET_DOWN_DEGREES)), 0);
        assertEquals(Noise.LOOP_SECONDS, built.value(Constant.LOOP_SECONDS), 0);
        assertEquals(Noise.LOOP_SPREAD, built.value(Constant.LOOP_SPREAD), 0);
        assertEquals(Noise.LEAST_LOOP_SECONDS, built.value(Constant.LEAST_LOOP_SECONDS), 0);
        assertEquals(Noise.HICCUP_CHANCE, built.value(Constant.HICCUP_CHANCE), 0);
        assertEquals(Noise.SHORTEST_HICCUP_SECONDS, built.value(Constant.SHORTEST_HICCUP_SECONDS), 0);
        assertEquals(Noise.LONGEST_HICCUP_SECONDS, built.value(Constant.LONGEST_HICCUP_SECONDS), 0);
        assertEquals(Launch.IN_PER_S_PER_TICK_PER_S, built.value(Constant.LAUNCH_THROW), 0);
        assertEquals(Flywheel.TicksPerTurn.BUILT, built.value(Constant.FLYWHEEL_TICKS_PER_TURN), 0);
        assertEquals(Turntable.TICKS_PER_SECOND_AT_FULL_POWER, built.value(Constant.TURNTABLE_SPEED), 0);

        for (Constant constant : Constant.values()) {
            assertEquals(constant.name(), constant.byDefault(), built.value(constant), 0);
        }
        assertTrue("nothing changed: " + built.changed(), built.changed().isEmpty());
        assertTrue(built.asBuilt());
    }

    @Test
    public void everyConstantSaysWhatItIsInWhatUnitAndWhereItMayGo() {
        for (Constant constant : Constant.values()) {
            assertFalse(constant.name(), constant.label().isBlank());
            assertFalse(constant.name(), constant.unit().isBlank());
            assertFalse(constant.name(), constant.says().isBlank());
            assertFalse(constant.name(), constant.group().label().isBlank());
            assertFalse(constant.name(), constant.group().says().isBlank());
            assertTrue(constant.name() + " is at least its least", constant.byDefault() >= constant.least());
            assertTrue(constant.name() + " is at most its most", constant.byDefault() <= constant.most());
            assertTrue(constant.name() + " has room to move", constant.least() < constant.most());
        }
    }

    @Test
    public void aConstantIsNamedByWhatItIsAskedForAsAndByNothingElse() {
        for (Constant constant : Constant.values()) {
            assertSame(constant, Valid.value(Constant.named(constant.asked())));
        }
        assertEquals("launch_throw", Constant.LAUNCH_THROW.asked());
        assertRejected("launch_throw", Constant.named("gravity"));
        assertRejected("'gravity'", Constant.named("gravity"));
        assertRejected("'LAUNCH_THROW'", Constant.named("LAUNCH_THROW"));
    }

    @Test
    public void theFlywheelsTicksPerTurnIsBuiltAtItsMotorsTicksATurnForTheFlywheelIsDirectDriveOnTheMotor() {
        assertEquals(
                Flywheel.TicksPerTurn.BUILT_SOURCE,
                28,
                Constants.defaults().value(Constant.FLYWHEEL_TICKS_PER_TURN),
                0);
        assertEquals(Flywheel.TicksPerTurn.BUILT_SOURCE, Flywheel.MOTOR_TICKS_PER_TURN, Flywheel.TicksPerTurn.BUILT, 0);
    }

    @Test
    public void theFlywheelsTicksPerTurnIsAMechanismThatGoesWhereverAFlywheelsTicksPerTurnMayAndSaysWhatItSets() {
        Constant ticks = Valid.value(Constant.named("flywheel_ticks_per_turn"));

        assertEquals(Constant.Group.MECHANISMS, ticks.group());
        assertEquals("Flywheel ticks per turn", ticks.label());
        assertEquals("ticks per turn of the flywheel", ticks.unit());
        assertEquals(
                "How many encoder ticks Reginald's flywheel motor counts as the flywheel turns once, gearing included:"
                        + " the flywheel turns at its motor's ticks a second over this, and the throw and spin follow"
                        + " from the simulated hood.",
                ticks.says());
        assertEquals(Flywheel.TicksPerTurn.RANGE_SOURCE, Flywheel.TicksPerTurn.LEAST, ticks.least(), 0);
        assertEquals(Flywheel.TicksPerTurn.RANGE_SOURCE, Flywheel.TicksPerTurn.MOST, ticks.most(), 0);
        for (double admitted : List.of(ticks.least(), ticks.byDefault(), ticks.most())) {
            assertEquals(
                    admitted,
                    Valid.value(Flywheel.TicksPerTurn.of(
                                    Valid.value(of(ticks, admitted)).value(ticks)))
                            .value(),
                    0);
        }
    }

    @Test
    public void aValueOutsideItsRangeIsRefusedSayingWhichConstantAndWhatItsRangeIs() {
        for (Constant constant : Constant.values()) {
            for (double wrong : List.of(
                    Math.nextDown(constant.least()),
                    Math.nextUp(constant.most()),
                    Double.NaN,
                    Double.POSITIVE_INFINITY,
                    Double.NEGATIVE_INFINITY)) {
                Checked<Constants> refused = of(constant, wrong);
                assertRejected(constant.label(), refused);
                assertRejected(String.valueOf(constant.least()), refused);
                assertRejected(String.valueOf(constant.most()), refused);
                assertRejected(String.valueOf(wrong), refused);
            }
        }
    }

    @Test
    public void aValueAtEitherEndOfItsRangeIsTaken() {
        for (Constant constant : List.of(
                Constant.MOTOR_SPREAD,
                Constant.LAUNCH_THROW,
                Constant.FLYWHEEL_TICKS_PER_TURN,
                Constant.TURNTABLE_SPEED)) {
            assertEquals(
                    constant.least(),
                    Valid.value(of(constant, constant.least())).value(constant),
                    0);
            assertEquals(
                    constant.most(), Valid.value(of(constant, constant.most())).value(constant), 0);
        }
    }

    @Test
    public void theEndsOfARangeASeedDrawsFromMustBeTheRightWayRound() {
        assertRejected("Flattest battery", of(Constant.FLATTEST_VOLTS, 13.9));
        assertRejected("freshest battery", of(Constant.FLATTEST_VOLTS, 13.9));
        assertRejected("13.9 V against 13.8 V", of(Constant.FLATTEST_VOLTS, 13.9));
        assertRejected("Least traction", of(Constant.LEAST_TRACTION_G, 0.7));
        assertRejected("Shortest loop", of(Constant.LEAST_LOOP_SECONDS, 0.04));
        assertRejected("Loop period", of(Constant.LOOP_SECONDS, 0.09));
        assertRejected("Shortest hiccup", of(Constant.SHORTEST_HICCUP_SECONDS, 0.3));

        assertEquals(
                13,
                Valid.value(of(Constant.FLATTEST_VOLTS, 13, Constant.FRESHEST_VOLTS, 13))
                        .value(Constant.FRESHEST_VOLTS),
                0);
    }

    @Test
    public void bothEndsOfARangeMoveTogetherInOneSet() {
        Constants grippier = Valid.value(of(Constant.LEAST_TRACTION_G, 0.7, Constant.MOST_TRACTION_G, 0.9));

        assertEquals(0.7, grippier.value(Constant.LEAST_TRACTION_G), 0);
        assertEquals(0.9, grippier.value(Constant.MOST_TRACTION_G), 0);
    }

    @Test
    public void whatChangedIsWhatDiffersFromTheValueAsBuiltInTheOrderTheyAreListed() {
        Constants changed = Valid.value(of(
                Constant.LAUNCH_THROW,
                0.2,
                Constant.MOTOR_SPREAD,
                Noise.MOTOR_SPREAD,
                Constant.SAG_VOLTS_PER_POWER,
                0.3));

        assertEquals(
                List.of(Constant.SAG_VOLTS_PER_POWER, Constant.LAUNCH_THROW),
                List.copyOf(changed.changed().keySet()));
        assertEquals(0.3, changed.changed().get(Constant.SAG_VOLTS_PER_POWER), 0);
        assertEquals(0.2, changed.changed().get(Constant.LAUNCH_THROW), 0);
        assertFalse(changed.asBuilt());
        assertEquals(Noise.MOTOR_SPREAD, changed.value(Constant.MOTOR_SPREAD), 0);
        assertEquals(Noise.LOOP_SECONDS, changed.value(Constant.LOOP_SECONDS), 0);
    }

    @Test
    public void twoSetsOfTheSameValuesAreTheSame() {
        assertEquals(Constants.defaults(), Valid.value(of()));
        assertEquals(Constants.defaults(), Valid.value(of(Constant.LAUNCH_THROW, Launch.IN_PER_S_PER_TICK_PER_S)));
        assertEquals(Valid.value(of(Constant.LAUNCH_THROW, 0.2)), Valid.value(of(Constant.LAUNCH_THROW, 0.2)));
        assertEquals(
                Valid.value(of(Constant.LAUNCH_THROW, 0.2)).hashCode(),
                Valid.value(of(Constant.LAUNCH_THROW, 0.2)).hashCode());
        assertFalse(Constants.defaults().equals(Valid.value(of(Constant.LAUNCH_THROW, 0.2))));
    }

    @Test
    public void whatASetIsHandedIsNotWhatItHolds() {
        Map<Constant, Double> handed = set(Constant.LAUNCH_THROW, 0.2);
        Constants constants = Valid.value(Constants.of(handed));
        handed.put(Constant.LAUNCH_THROW, 0.5);
        constants.changed().put(Constant.LAUNCH_THROW, 0.7);

        assertEquals(0.2, constants.value(Constant.LAUNCH_THROW), 0);
    }

    @Test
    public void aSavedValuePastItsRangeIsRestoredAtTheNearerEndAndWhatWasSavedIsKeptToSay() {
        Constants.Restored restored = Valid.value(Constants.restored(
                set(Constant.LAUNCH_THROW, 0.5, Constant.MOTOR_SPREAD, -0.1, Constant.HICCUP_CHANCE, 0.5)));

        assertEquals(Constant.LAUNCH_THROW.most(), restored.constants().value(Constant.LAUNCH_THROW), 0);
        assertEquals(Constant.MOTOR_SPREAD.least(), restored.constants().value(Constant.MOTOR_SPREAD), 0);
        assertEquals(0.5, restored.constants().value(Constant.HICCUP_CHANCE), 0);
        assertEquals(set(Constant.MOTOR_SPREAD, -0.1, Constant.LAUNCH_THROW, 0.5), restored.pastTheirRange());
        assertEquals(
                List.of(Constant.MOTOR_SPREAD, Constant.LAUNCH_THROW),
                List.copyOf(restored.pastTheirRange().keySet()));
    }

    @Test
    public void savedValuesWithinTheirRangesAreRestoredAsTheyWereSavedAndNothingIsSaidOfThem() {
        Map<Constant, Double> saved = set(Constant.LAUNCH_THROW, 0.25, Constant.HICCUP_CHANCE, 0);

        Constants.Restored restored = Valid.value(Constants.restored(saved));

        assertEquals(Valid.value(Constants.of(saved)), restored.constants());
        assertTrue(
                restored.pastTheirRange().toString(), restored.pastTheirRange().isEmpty());
        assertEquals(
                Constants.defaults(), Valid.value(Constants.restored(set())).constants());
        assertTrue(
                "the least end itself, however it is signed",
                Valid.value(Constants.restored(set(Constant.MOTOR_SPREAD, -0.0)))
                        .pastTheirRange()
                        .isEmpty());
    }

    @Test
    public void aSavedValueThatIsNoNumberOrPutsARangesEndsTheWrongWayRoundIsNotRestoredAndSaysWhy() {
        for (double wrong : List.of(Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY)) {
            Checked<Constants.Restored> refused = Constants.restored(set(Constant.LAUNCH_THROW, wrong));
            assertRejected(Constant.LAUNCH_THROW.label(), refused);
            assertRejected(String.valueOf(wrong), refused);
        }
        assertRejected(
                "Least traction may not be more than most traction",
                Constants.restored(set(Constant.LEAST_TRACTION_G, 0.9)));
    }

    @Test
    public void whatARestoredSetSaysWasSavedIsNotWhatItHolds() {
        Constants.Restored restored = Valid.value(Constants.restored(set(Constant.LAUNCH_THROW, 0.5)));

        restored.pastTheirRange().put(Constant.LAUNCH_THROW, 0.7);
        restored.asSaved().put(Constant.LAUNCH_THROW, 0.7);

        assertEquals(set(Constant.LAUNCH_THROW, 0.5), restored.pastTheirRange());
        assertEquals(set(Constant.LAUNCH_THROW, 0.5), restored.asSaved());
    }

    @Test
    public void aRestoredSetSavesWhatWasSavedSoWhatItSavesIsRestoredAsTheSameSet() {
        Map<Constant, Double> saved =
                set(Constant.MOTOR_SPREAD, -0.1, Constant.HICCUP_CHANCE, 0.5, Constant.LAUNCH_THROW, 0.5);
        Constants.Restored restored = Valid.value(Constants.restored(saved));

        assertEquals(saved, restored.asSaved());
        Constants.Restored again = Valid.value(Constants.restored(restored.asSaved()));
        assertEquals(restored.constants(), again.constants());
        assertEquals(restored.pastTheirRange(), again.pastTheirRange());
    }

    @Test
    public void aSetSetAsItIsSavesWhatDiffersFromHowItWasBuiltAndHasNothingSavedPastItsRange() {
        Constants set = Valid.value(of(Constant.LAUNCH_THROW, 0.25, Constant.HICCUP_CHANCE, 0));

        Constants.Restored asSet = Constants.Restored.asSet(set);

        assertEquals(set, asSet.constants());
        assertTrue(asSet.pastTheirRange().toString(), asSet.pastTheirRange().isEmpty());
        assertEquals(set(Constant.HICCUP_CHANCE, 0, Constant.LAUNCH_THROW, 0.25), asSet.asSaved());
        assertTrue(Constants.Restored.asSet(Constants.defaults()).asSaved().isEmpty());
    }

    @Test
    public void theFastestBallTheLauncherThrowConstantAllowsFromTheFastestRobotIsOneAFlightTakes() {
        assertTrue(
                Constant.LAUNCH_THROW.most() + " in/s per tick/s",
                Constant.LAUNCH_THROW.most() * Flywheel.TOP_TICKS_PER_SECOND + Launch.MOST_ROBOT_SPEED_IN_PER_S
                        <= Flight.MOST_SPEED_IN_PER_S);
    }
}
