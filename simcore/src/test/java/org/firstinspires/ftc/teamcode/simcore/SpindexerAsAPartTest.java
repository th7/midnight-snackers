package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.Test;

public class SpindexerAsAPartTest {
    private static final double MILLIMETRE = 1 / 25.4;
    private static final double R = Launcher.POLLEN_RADIUS_IN;
    private static final Length POLLEN = Valid.length(R);
    private static final String PLACE_SOURCE = "doc/plans/spindexer-sim.md, What the CAD says, Placed in Reginald: the"
            + " spindexer frame maps to Reginald from (0, 173.96, 28.78) tilted 16.46 degrees, and the launcher's frame is"
            + " centred 204 mm behind Reginald's front, x forward, y left, z up";
    private static final double ROBOT_CENTRE_BACK_MM = 204;
    private static final double TILT = StrictMath.toRadians(16.46);
    private static final double BEGINS = 0.1;
    private static final double TICK = 0.004;
    private static final List<Double> TIMES = List.of(0.096, BEGINS, 0.1013, 0.102, BEGINS + TICK, 0.109);
    private static final double EXACTLY = 1e-9;
    private static final double WALL_MM = 82.27;
    private static final double FLOOR_MM = 15;

    private static Vec3 world(double xMm, double yMm, double zMm) {
        double cos = StrictMath.cos(TILT), sin = StrictMath.sin(TILT);
        double back = 173.96 - yMm * cos - zMm * sin;
        double up = 28.78 - yMm * sin + zMm * cos;
        return new Vec3((ROBOT_CENTRE_BACK_MM - back) * MILLIMETRE, -xMm * MILLIMETRE, up * MILLIMETRE);
    }

    private static Vec3 worldDirection(double x, double y, double z) {
        return world(x, y, z).minus(world(0, 0, 0)).times(1 / MILLIMETRE);
    }

    private static Spindexer.Shape cad() {
        return Valid.value(Spindexer.Shape.cad());
    }

    private static Launcher.Frame atTheCentre() {
        return Valid.value(Launcher.Frame.of(0, 0, 0));
    }

    private static Spindexer.Spinner.Turning turning(double revolutionsPerSecond) {
        return Spindexer.Spinner.revSmartRobotServo()
                .next(
                        Spindexer.Spinner.Turning.still(),
                        Valid.value(Power.of(revolutionsPerSecond)),
                        Valid.value(Spindexer.Torque.ofNewtonMetres(0)),
                        Valid.value(Seconds.of(TICK)));
    }

    private static Spindexer.RampAngle ramp(double degrees) {
        return Valid.value(Spindexer.RampAngle.ofDegrees(degrees));
    }

    private static Spindexer atRest() {
        return Valid.value(Spindexer.placed(
                cad(),
                atTheCentre(),
                Spindexer.Motion.still(Valid.time(BEGINS), Spindexer.Spinner.Turning.still(), ramp(0))));
    }

    private static Spindexer moving(
            Spindexer.Spinner.Turning from,
            Spindexer.Spinner.Turning to,
            Spindexer.RampAngle down,
            Spindexer.RampAngle up) {
        return Valid.value(Spindexer.placed(
                cad(),
                atTheCentre(),
                Valid.value(
                        Spindexer.Motion.of(Valid.time(BEGINS), Valid.value(Seconds.of(TICK)), from, to, down, up))));
    }

    private static Spindexer spinning() {
        return moving(Spindexer.Spinner.Turning.still(), turning(1), ramp(0), ramp(0));
    }

    private static Spindexer raising() {
        return moving(Spindexer.Spinner.Turning.still(), Spindexer.Spinner.Turning.still(), ramp(30), ramp(31));
    }

    private static List<PartContract.Probe> probes() {
        List<PartContract.Probe> probes = new ArrayList<>();
        for (double x = -88; x <= 88; x += 11.3) {
            for (double y = -95; y <= 88; y += 10.7) {
                if (StrictMath.hypot(x, y) > 100) {
                    continue;
                }
                for (double z = 8; z <= 125; z += 9.1) {
                    for (double within : new double[] {0, Shot.TOUCHING_IN, 0.4}) {
                        probes.add(PartContract.Probe.of(world(x, y, z), R, within));
                    }
                }
            }
        }
        return probes;
    }

    private static Optional<Surroundings.Touch> named(List<Surroundings.Touch> touches, String id) {
        Optional<Surroundings.Touch> found = Optional.empty();
        for (Surroundings.Touch touch : touches) {
            if (touch.id().equals(id)) {
                assertTrue("one touch for " + id + " among " + touches, found.isEmpty());
                found = Optional.of(touch);
            }
        }
        return found;
    }

    private static void assertDirection(String what, Vec3 expected, Vec3 actual) {
        assertEquals(what + ": " + actual, expected.x(), actual.x(), EXACTLY);
        assertEquals(what + ": " + actual, expected.y(), actual.y(), EXACTLY);
        assertEquals(what + ": " + actual, expected.z(), actual.z(), EXACTLY);
    }

    @Test
    public void itsNamesAreTheFloorTheWallEachWindowsSillAndSidesTheTopPlateTheSpinnersHubAndArmsAndTheRampBar() {
        assertEquals(
                List.of(
                        "Spindexer floor",
                        "Spindexer wall",
                        "Spindexer front sill",
                        "Spindexer front window's left side",
                        "Spindexer front window's right side",
                        "Spindexer back sill",
                        "Spindexer back window's left side",
                        "Spindexer back window's right side",
                        "Spindexer top plate",
                        "Spinner hub",
                        "Spinner's first arm",
                        "Spinner's second arm",
                        "Ramp bar"),
                atRest().ids());
    }

    @Test
    public void itKeepsThePartContractAtRestWithTheSpinnerTurningAndWithTheRampMoving() {
        List<PartContract.Probe> probes = probes();
        PartContract.assertKept("at rest", atRest(), probes, TIMES, (seconds, touch) -> true);
        PartContract.assertKept(
                "spinner turning",
                spinning(),
                probes,
                TIMES,
                (seconds, touch) -> !touch.id().startsWith("Spinner"));
        PartContract.assertKept(
                "ramp moving",
                raising(),
                probes,
                TIMES,
                (seconds, touch) -> !touch.id().equals("Ramp bar"));
    }

    @Test
    public void aPollenSeatedOnTheFloorAgainstTheWallTouchesBothAlongTheSpindexersAxisAndInwardExactly() {
        double centre = WALL_MM - R / MILLIMETRE;
        Vec3 seated = world(centre, 0, FLOOR_MM + R / MILLIMETRE);
        List<Surroundings.Touch> touches = atRest().near(seated, POLLEN, Shot.TOUCHING_IN);

        Surroundings.Touch floor = named(touches, "Spindexer floor").orElseThrow(() -> new AssertionError(touches));
        Surroundings.Touch wall = named(touches, "Spindexer wall").orElseThrow(() -> new AssertionError(touches));
        assertEquals(PLACE_SOURCE, 0, floor.gap(), EXACTLY);
        assertEquals(PLACE_SOURCE, 0, wall.gap(), EXACTLY);
        assertDirection("up the spindexer's axis", worldDirection(0, 0, 1), floor.normal());
        assertDirection("toward the robot's left, the axis", worldDirection(-1, 0, 0), wall.normal());
        assertTrue("still", !floor.moves() && !wall.moves());
    }

    @Test
    public void anArmTurningClockwiseAtOneTurnASecondMovesItsFaceAtItsDistanceFromTheAxisTimesTwoPi() {
        Spindexer spindexer = spinning().at(Valid.time(BEGINS));
        Vec3 against = world(2 + R / MILLIMETRE, 30, 50);
        List<Surroundings.Touch> touches = spindexer.near(against, POLLEN, Shot.TOUCHING_IN);

        Surroundings.Touch arm = named(touches, "Spinner's first arm").orElseThrow(() -> new AssertionError(touches));
        assertEquals(0, arm.gap(), EXACTLY);
        assertDirection("off the arm's side toward the robot's right", worldDirection(1, 0, 0), arm.normal());
        assertEquals("2 pi 30 mm a second, in inches", 7.421085, arm.velocity().dot(arm.normal()), 1e-6);
        assertDirection("turning clockwise seen from above", worldDirection(0, 0, -2 * StrictMath.PI), arm.spin());
    }

    @Test
    public void theRampBarRaisingADegreeEachTickMovesItsTopAlongItsNormalAtItsDistanceAlongFromThePivot() {
        Spindexer spindexer = raising().at(Valid.time(BEGINS));
        double turn = StrictMath.toRadians(30);
        double topX = 57.54 + (0 - 57.54) * StrictMath.cos(turn) + (-11.49 + 13.99) * StrictMath.sin(turn);
        double topZ = Valid.value(cad().barTopMm(0, ramp(30)));
        double off = R / MILLIMETRE;
        Vec3 onTheBar = world(topX + off * StrictMath.sin(turn), -53.95, topZ + off * StrictMath.cos(turn));
        List<Surroundings.Touch> touches = spindexer.near(onTheBar, POLLEN, Shot.TOUCHING_IN);

        Surroundings.Touch bar = named(touches, "Ramp bar").orElseThrow(() -> new AssertionError(touches));
        assertEquals(0, bar.gap(), 1e-9);
        assertDirection(
                "square to the bar's top", worldDirection(StrictMath.sin(turn), 0, StrictMath.cos(turn)), bar.normal());
        assertEquals(
                "a degree in 4 ms, 57.54 mm along from the pivot, in inches a second",
                9.88447,
                bar.velocity().dot(bar.normal()),
                1e-4);
    }
}
