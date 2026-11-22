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
        assertEquals("set_down_degrees", Constant.SET_DOWN_DEGREES.asked());
        assertRejected("set_down_degrees", Constant.named("gravity"));
        assertRejected("'gravity'", Constant.named("gravity"));
        assertRejected("'SET_DOWN_DEGREES'", Constant.named("SET_DOWN_DEGREES"));
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
        for (Constant constant : List.of(Constant.MOTOR_SPREAD, Constant.SET_DOWN_DEGREES, Constant.LOOP_SPREAD)) {
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
                Constant.SET_DOWN_DEGREES,
                20,
                Constant.MOTOR_SPREAD,
                Noise.MOTOR_SPREAD,
                Constant.SAG_VOLTS_PER_POWER,
                0.3));

        assertEquals(
                List.of(Constant.SAG_VOLTS_PER_POWER, Constant.SET_DOWN_DEGREES),
                List.copyOf(changed.changed().keySet()));
        assertEquals(0.3, changed.changed().get(Constant.SAG_VOLTS_PER_POWER), 0);
        assertEquals(20, changed.changed().get(Constant.SET_DOWN_DEGREES), 0);
        assertFalse(changed.asBuilt());
        assertEquals(Noise.MOTOR_SPREAD, changed.value(Constant.MOTOR_SPREAD), 0);
        assertEquals(Noise.LOOP_SECONDS, changed.value(Constant.LOOP_SECONDS), 0);
    }

    @Test
    public void twoSetsOfTheSameValuesAreTheSame() {
        assertEquals(Constants.defaults(), Valid.value(of()));
        assertEquals(
                Constants.defaults(),
                Valid.value(of(Constant.SET_DOWN_DEGREES, Constant.SET_DOWN_DEGREES.byDefault())));
        assertEquals(Valid.value(of(Constant.SET_DOWN_DEGREES, 20)), Valid.value(of(Constant.SET_DOWN_DEGREES, 20)));
        assertEquals(
                Valid.value(of(Constant.SET_DOWN_DEGREES, 20)).hashCode(),
                Valid.value(of(Constant.SET_DOWN_DEGREES, 20)).hashCode());
        assertFalse(Constants.defaults().equals(Valid.value(of(Constant.SET_DOWN_DEGREES, 20))));
    }

    @Test
    public void whatASetIsHandedIsNotWhatItHolds() {
        Map<Constant, Double> handed = set(Constant.SET_DOWN_DEGREES, 20);
        Constants constants = Valid.value(Constants.of(handed));
        handed.put(Constant.SET_DOWN_DEGREES, 30.0);
        constants.changed().put(Constant.SET_DOWN_DEGREES, 40.0);

        assertEquals(20, constants.value(Constant.SET_DOWN_DEGREES), 0);
    }
}
