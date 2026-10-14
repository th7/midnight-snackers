package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.junit.Test;

public class LauncherAsAPartTest {
    private static final double MILLIMETRE = 1 / 25.4;
    private static final double DEGREE = StrictMath.PI / 180;
    private static final double R = 1.39;
    private static final Length POLLEN = Valid.length(R);
    private static final Ball BALL = Valid.value(Ball.of(POLLEN, 24.9, 0.634));
    private static final Vec3 HUB = new Vec3(-13 * MILLIMETRE, 0, 180 * MILLIMETRE);
    private static final List<Double> RINGS_MM = List.of(-25.0, -12.5, 0.0, 12.5, 25.0);
    private static final double TICKS_PER_TURN = 24.2;
    private static final double CLOSE_LAUNCH = 1050;
    private static final double BEGINS = 0.1;
    private static final double ENDS = 0.105;
    private static final List<Double> TIMES = List.of(0.0, 0.09, BEGINS, 0.1025, ENDS, 0.12);
    private static final double HOOD_FROM = -64.8;
    private static final double EXTENSION_FROM = -73;
    private static final double LEAD_IN_SQUEEZE = 0.25 * MILLIMETRE;
    private static final double FEED_IN_PER_S = 0.1 / Length.METRES_PER_INCH;
    private static final String STOP = "Launcher stop";
    private static final double WITHIN = 1e-9;
    private static final Colliders FIELD = Valid.value(Colliders.of(
            Valid.value(Field.of(
                    Valid.length(141.17),
                    Valid.length(11.64),
                    List.of(),
                    List.of(),
                    List.of(),
                    List.of(),
                    List.of(),
                    List.of())),
            List.of()));

    private static double wheelRing(double millimetres) {
        return (75.2886 - StrictMath.sqrt(35.56 * 35.56 - millimetres * millimetres)) * MILLIMETRE;
    }

    private static double hoodRing(double millimetres) {
        return (72.341 + StrictMath.sqrt(36 * 36 - millimetres * millimetres)) * MILLIMETRE;
    }

    private static Vec3 outward(double degrees) {
        return new Vec3(-StrictMath.cos(degrees * DEGREE), 0, StrictMath.sin(degrees * DEGREE));
    }

    private static Vec3 alongThePath(double degrees) {
        return new Vec3(StrictMath.sin(degrees * DEGREE), 0, StrictMath.cos(degrees * DEGREE));
    }

    private static Vec3 onThePath(double degrees, double out) {
        return HUB.plus(outward(degrees).times(out));
    }

    private static String wheelId(double millimetres) {
        return "Flywheel ring at " + millimetres + " mm";
    }

    private static String hoodId(double millimetres) {
        return "Hood ring at " + millimetres + " mm";
    }

    private static Launcher.Faces faces() {
        return Valid.value(Launcher.Faces.built());
    }

    private static Flywheel.AngularSpeed wheel(double ticksPerSecond) {
        return wheel(ticksPerSecond, TICKS_PER_TURN);
    }

    private static Flywheel.AngularSpeed wheel(double ticksPerSecond, double ticksPerTurn) {
        return Flywheel.AngularSpeed.from(ticksPerSecond, Valid.value(Flywheel.TicksPerTurn.of(ticksPerTurn)));
    }

    private static Launcher.Frame frame(double x, double y, double yaw) {
        return Valid.value(Launcher.Frame.of(x, y, yaw));
    }

    private static Launcher.Course course(Launcher.Frame from, Launcher.Frame to) {
        return Valid.value(Launcher.Course.of(from, Valid.time(BEGINS), to, Valid.time(ENDS)));
    }

    private static Launcher launcher(Launcher.Course course, Flywheel.AngularSpeed wheel) {
        return Launcher.placed(faces(), course, wheel);
    }

    private static Launcher still(double ticksPerSecond) {
        return launcher(course(frame(0, 0, 0), frame(0, 0, 0)), wheel(ticksPerSecond));
    }

    private static Optional<Surroundings.Overlap> named(List<Surroundings.Overlap> overlaps, String id) {
        Optional<Surroundings.Overlap> found = Optional.empty();
        for (Surroundings.Overlap overlap : overlaps) {
            if (overlap.id().equals(id)) {
                assertTrue("one overlap for " + id + " among " + overlaps, found.isEmpty());
                found = Optional.of(overlap);
            }
        }
        return found;
    }

    private static double depthOn(Launcher launcher, Vec3 centre, String id) {
        return named(launcher.squeezing(centre, POLLEN), id)
                .map(Surroundings.Overlap::depth)
                .orElse(0.0);
    }

    private static double offTheCircle(Vec3 point, double ringMillimetres, double radius) {
        Vec3 off = point.minus(HUB);
        double out = StrictMath.hypot(off.x(), off.z());
        return StrictMath.hypot(out - radius, off.y() - ringMillimetres * MILLIMETRE);
    }

    private static Compliance wheelSurface() {
        return Valid.value(Valid.value(Compliance.of(
                        Compliance.STIFFNESS_N_PER_MM,
                        Compliance.RESTITUTION,
                        Compliance.WHEEL_FRICTION,
                        Compliance.REGULARISATION_M_PER_S))
                .sharedBy(5));
    }

    private static Compliance hoodSurface() {
        return Valid.value(Valid.value(Compliance.of(
                        Compliance.STIFFNESS_N_PER_MM,
                        Compliance.RESTITUTION,
                        Compliance.HOOD_FRICTION,
                        Compliance.REGULARISATION_M_PER_S))
                .sharedBy(5));
    }

    @Test
    public void itsFacesAreTheCadsWithTheHoodAndTheLeadInAsTheTeamAllowedThemChanged() {
        assertEquals(Launcher.CAD_SOURCE, 217 - 204, Launcher.AXLE_BEHIND_CENTRE_MM, 0);
        assertEquals(Launcher.CAD_SOURCE, 180, Launcher.AXLE_UP_MM, 0);
        assertEquals(Launcher.CAD_SOURCE, 75.2886, Launcher.WHEEL_TORUS_MM, 0);
        assertEquals(Launcher.CAD_SOURCE, 35.56, Launcher.WHEEL_TUBE_MM, 0);
        assertEquals(Launcher.CAD_SOURCE, 72.341, Launcher.HOOD_TORUS_MM, 0);
        assertEquals(Launcher.CAD_SOURCE, 36, Launcher.HOOD_TUBE_MM, 0);
        assertEquals(Launcher.CAD_SOURCE, 12.5, Launcher.RING_APART_MM, 0);
        assertEquals(Launcher.CAD_SOURCE, 5, Launcher.RINGS);
        assertEquals(Launcher.CAD_SOURCE, HOOD_FROM, Launcher.HOOD_FROM_DEGREES, 0);
        assertEquals(Launcher.CAD_SOURCE, 20, Launcher.HOOD_TO_DEGREES, 0);
        assertEquals(Launcher.CAD_SOURCE, EXTENSION_FROM, Launcher.EXTENSION_FROM_DEGREES, 0);
        assertEquals(Launcher.CAD_SOURCE, 0.25, Launcher.LEAD_IN_SQUEEZE_MM, 0);
        assertEquals(Launcher.POLLEN_SOURCE, R, Launcher.POLLEN_RADIUS_IN, 0);
        assertTrue(
                Launcher.STOP_SOURCE,
                still(0).near(still(0).entry(POLLEN), POLLEN, 0).get(0).surface() == Surface.FRAME);
    }

    @Test
    public void itsNamesAreItsFiveFlywheelRingsItsFiveHoodRingsAndItsStopEachOnce() {
        Set<String> expected = new HashSet<>();
        for (double ring : RINGS_MM) {
            expected.add(wheelId(ring));
            expected.add(hoodId(ring));
        }
        expected.add(STOP);
        List<String> ids = still(CLOSE_LAUNCH).ids();

        assertEquals(11, ids.size());
        assertEquals(expected, new HashSet<>(ids));
        assertTrue(Valid.value(Surroundings.of(FIELD, List.of(still(0)))).squeezes());
    }

    @Test
    public void itsRingsSampleTheFlywheelsAndTheHoodsGroovesAcrossTheirWidthAsThePlansTablesGiveThem() {
        Launcher launcher = still(CLOSE_LAUNCH);
        for (double degrees : new double[] {-60, -41.3, -20, 0, 7.5, 19}) {
            for (double sideways : new double[] {0, 0.13, -0.31}) {
                for (double nearer : new double[] {0, 0.5, 1.5}) {
                    Vec3 centre = onThePath(degrees, wheelRing(0) + R - nearer * MILLIMETRE)
                            .plus(new Vec3(0, sideways, 0));
                    List<Surroundings.Overlap> overlaps = launcher.squeezing(centre, POLLEN);
                    for (double ring : RINGS_MM) {
                        double wheel = R - offTheCircle(centre, ring, wheelRing(ring));
                        double hood = R - offTheCircle(centre, ring, hoodRing(ring));
                        String where = " at " + degrees + " degrees, " + sideways + " in across, " + nearer
                                + " mm nearer the wheel";
                        assertEquals(
                                wheelId(ring) + where,
                                StrictMath.max(0, wheel),
                                depthOn(launcher, centre, wheelId(ring)),
                                WITHIN);
                        assertEquals(
                                hoodId(ring) + where,
                                StrictMath.max(0, hood),
                                depthOn(launcher, centre, hoodId(ring)),
                                WITHIN);
                        named(overlaps, wheelId(ring))
                                .ifPresent(overlap -> assertEquals(wheelSurface(), overlap.compliance()));
                        named(overlaps, hoodId(ring))
                                .ifPresent(overlap -> assertEquals(hoodSurface(), overlap.compliance()));
                    }
                }
            }
        }
    }

    @Test
    public void itsHoodEndsAtItsUpperEndBeyondWhichABallIsNearestThatEdge() {
        Launcher launcher = still(0);
        double upper = Launcher.HOOD_TO_DEGREES;
        Vec3 edge = HUB.plus(outward(upper).times(hoodRing(0)));
        for (double past : new double[] {1, 3, 6}) {
            Vec3 centre = onThePath(upper + past, wheelRing(0) + R);
            Vec3 apart = centre.minus(edge);
            double depth = R - StrictMath.sqrt(apart.dot(apart));
            assertEquals(
                    "past the upper end by " + past,
                    StrictMath.max(0, depth),
                    depthOn(launcher, centre, hoodId(0)),
                    WITHIN);
        }
        Vec3 within = onThePath(upper - 1, wheelRing(0) + R);
        assertEquals(R - offTheCircle(within, 0, hoodRing(0)), depthOn(launcher, within, hoodId(0)), WITHIN);
    }

    @Test
    public void
            itsLeadInOpensFromTheHoodsLowerEndToAGapOfThePollensDiameterLessAQuarterMillimetreAtTheExtensionsLowerEnd() {
        Launcher launcher = still(0);
        double full = 2 * R - (hoodRing(0) - wheelRing(0));
        double opening = (2 * R - LEAD_IN_SQUEEZE) - (hoodRing(0) - wheelRing(0));
        double previous = Double.POSITIVE_INFINITY;
        for (double degrees = HOOD_FROM; degrees >= EXTENSION_FROM - 1e-9; degrees -= 0.41) {
            Vec3 centre = onThePath(degrees, wheelRing(0) + R);
            double depth = depthOn(launcher, centre, hoodId(0));
            assertTrue("at " + degrees + " squeezed " + depth / MILLIMETRE + " mm", depth < previous + 1e-12);
            previous = depth;
        }
        assertEquals(full, depthOn(launcher, onThePath(HOOD_FROM, wheelRing(0) + R), hoodId(0)), WITHIN);
        assertTrue(depthOn(launcher, onThePath(EXTENSION_FROM, wheelRing(0) + R), hoodId(0)) < full);
        for (double ring : RINGS_MM) {
            Vec3 across = new Vec3(0, ring * MILLIMETRE, 0);
            Vec3 lowerEnd = HUB.plus(across).plus(outward(EXTENSION_FROM).times(hoodRing(ring) + opening));
            for (double degrees : new double[] {-81, -83, -86}) {
                Vec3 centre = HUB.plus(across).plus(outward(degrees).times(hoodRing(ring) + opening - 30 * MILLIMETRE));
                Vec3 apart = centre.minus(lowerEnd);
                double depth = R - StrictMath.sqrt(apart.dot(apart));
                assertEquals(
                        "below the lead-in of " + ring + " at " + degrees,
                        StrictMath.max(0, depth),
                        depthOn(launcher, centre, hoodId(ring)),
                        WITHIN);
                assertTrue("below the lead-in of " + ring, degrees < -81 || depth > 0);
            }
        }
    }

    @Test
    public void aBallFedAtTheEntrySitsOneRadiusOffTheGrooveBottomTouchingTheStopBelowItAndGoesUpThePathAtTheFeed() {
        Launcher launcher = still(CLOSE_LAUNCH);
        Vec3 entry = onThePath(EXTENSION_FROM, wheelRing(0) + R);
        assertEquals(entry.x(), launcher.entry(POLLEN).x(), WITHIN);
        assertEquals(entry.y(), launcher.entry(POLLEN).y(), WITHIN);
        assertEquals(entry.z(), launcher.entry(POLLEN).z(), WITHIN);
        assertEquals(Launcher.FEED_SOURCE, 0.1, Launcher.FEED_M_PER_S, 0);
        assertTrue(Launcher.FEED_SOURCE, Launcher.FEED_SOURCE.contains("reginald-internals.md"));
        Vec3 feed = alongThePath(EXTENSION_FROM).times(FEED_IN_PER_S);
        assertEquals(feed.x(), launcher.feed(POLLEN).x(), WITHIN);
        assertEquals(feed.y(), launcher.feed(POLLEN).y(), WITHIN);
        assertEquals(feed.z(), launcher.feed(POLLEN).z(), WITHIN);

        List<Surroundings.Touch> touches = launcher.near(entry, POLLEN, Shot.TOUCHING_IN);
        assertEquals(1, touches.size());
        Surroundings.Touch stop = touches.get(0);
        assertEquals(STOP, stop.id());
        assertEquals(0, stop.gap(), WITHIN);
        Vec3 up = alongThePath(EXTENSION_FROM);
        assertEquals(1, stop.normal().dot(up), WITHIN);
        Vec3 down = entry.minus(up.times(0.1));
        assertEquals(-0.1, launcher.near(down, POLLEN, 0).get(0).gap(), WITHIN);
        assertEquals(0, depthOn(launcher, entry, wheelId(0)), WITHIN);
        assertTrue(depthOn(launcher, entry, hoodId(0)) > LEAD_IN_SQUEEZE);
        assertEquals(List.of(), launcher.near(entry.plus(up.times(0.2)), POLLEN, Shot.TOUCHING_IN));
    }

    @Test
    public void aStillLauncherWithItsWheelStoppedSqueezesAndTouchesWithNothingMovingExactly() {
        List<Launcher> resting = List.of(
                still(0),
                still(Double.NaN),
                launcher(course(frame(-31.7, 12.25, 2.9), frame(-31.7, 12.25, 2.9)), wheel(0)),
                Launcher.placed(
                        faces(),
                        Valid.value(Launcher.Course.of(
                                frame(5, 5, 1), Valid.time(ENDS), frame(-31.7, 12.25, 2.9), Valid.time(ENDS))),
                        wheel(-0.0)));
        int squeezed = 0, touched = 0;
        for (Launcher launcher : resting) {
            for (double t : TIMES) {
                Launcher then = launcher.at(Valid.time(t));
                assertTrue(!then.moving());
                assertEquals(0, then.sweep(), 0);
                for (PartContract.Probe probe : probes(then)) {
                    for (Surroundings.Overlap overlap : then.squeezing(probe.centre(), probe.radius())) {
                        Surroundings.Squeeze squeeze =
                                Valid.value(Surroundings.Squeeze.of(overlap, probe.centre(), probe.radius()));
                        assertExactlyZero(overlap.id(), squeeze.velocity());
                        assertExactlyZero(overlap.id(), squeeze.spin());
                        squeezed++;
                    }
                    for (Surroundings.Touch touch : then.near(probe.centre(), probe.radius(), probe.within())) {
                        assertExactlyZero(touch.id(), touch.velocity());
                        assertExactlyZero(touch.id(), touch.spin());
                        touched++;
                    }
                }
            }
        }
        assertTrue("squeezed " + squeezed + " times and touched " + touched, squeezed > 1000 && touched > 10);
    }

    private static void assertExactlyZero(String what, Vec3 v) {
        assertTrue(what + " moves at " + v, v.x() == 0 && v.y() == 0 && v.z() == 0);
    }

    private static Vec3 turned(Vec3 local, double yaw) {
        double cos = StrictMath.cos(yaw), sin = StrictMath.sin(yaw);
        return new Vec3(cos * local.x() - sin * local.y(), sin * local.x() + cos * local.y(), local.z());
    }

    private static Vec3 placedAt(Launcher.Placed placed, Vec3 local) {
        return placed.origin().plus(turned(local, placed.yaw()));
    }

    private static void assertVec(String what, Vec3 expected, Vec3 actual, double within) {
        assertEquals(what + ": x of " + actual, expected.x(), actual.x(), within);
        assertEquals(what + ": y of " + actual, expected.y(), actual.y(), within);
        assertEquals(what + ": z of " + actual, expected.z(), actual.z(), within);
    }

    @Test
    public void aWheelRingMovesWithTheFrameAndTurnsWithTheWheelAboutTheHubAndAHoodRingMovesWithTheFrameAlone() {
        Launcher.Course course = course(frame(-20, 30, 3.0), frame(-19.6, 30.3, -3.1));
        double omega = wheel(CLOSE_LAUNCH).radiansPerSecond();
        for (double t : new double[] {BEGINS, 0.1031, ENDS, 0.13}) {
            Launcher launcher = launcher(course, wheel(CLOSE_LAUNCH)).at(Valid.time(t));
            Launcher.Placed placed = course.at(Valid.time(t));
            Vec3 frameSpin = new Vec3(0, 0, course.turning());
            Vec3 hub = placedAt(placed, HUB);
            Vec3 axle = turned(new Vec3(0, 1, 0), placed.yaw());
            for (double degrees : new double[] {-70, -40, 0, 18}) {
                Vec3 centre = placedAt(placed, onThePath(degrees, wheelRing(0) + R - 0.6 * MILLIMETRE));
                Set<String> squeezing = new HashSet<>();
                for (Surroundings.Overlap overlap : launcher.squeezing(centre, POLLEN)) {
                    squeezing.add(overlap.id());
                    Surroundings.Squeeze squeeze = Valid.value(Surroundings.Squeeze.of(overlap, centre, POLLEN));
                    Vec3 meets = centre.plus(squeeze.arm());
                    Vec3 withTheFrame = course.velocity().plus(frameSpin.cross(meets.minus(placed.origin())));
                    boolean ofTheWheel = overlap.id().startsWith("Flywheel");
                    Vec3 spin = ofTheWheel ? frameSpin.plus(axle.times(omega)) : frameSpin;
                    Vec3 velocity =
                            ofTheWheel ? withTheFrame.plus(axle.times(omega).cross(meets.minus(hub))) : withTheFrame;
                    assertVec(
                            overlap.id() + " at " + t + " s, " + degrees + " degrees",
                            velocity,
                            squeeze.velocity(),
                            1e-9);
                    assertVec(overlap.id() + " spins", spin, squeeze.spin(), 1e-12);
                }
                assertTrue(squeezing.contains(wheelId(0)) && squeezing.contains(hoodId(0)));
            }
        }
    }

    @Test
    public void
            aBallRollingWithTheWheelInAMovingTurningFrameIsNotRubbedByItAndOneCarriedWithTheFrameIsNotRubbedByTheHood() {
        Launcher.Course course = course(frame(12, -40, 1.0), frame(12.3, -40.2, 1.12));
        Launcher launcher = launcher(course, wheel(CLOSE_LAUNCH)).at(Valid.time(0.1037));
        Launcher.Placed placed = course.at(Valid.time(0.1037));
        Vec3 frameSpin = new Vec3(0, 0, course.turning());
        Vec3 hub = placedAt(placed, HUB);
        Vec3 wheelSpin = frameSpin.plus(turned(new Vec3(0, 1, 0), placed.yaw())
                .times(wheel(CLOSE_LAUNCH).radiansPerSecond()));
        for (double degrees : new double[] {-50, -10, 15}) {
            Vec3 centre = placedAt(placed, onThePath(degrees, wheelRing(0) + R - 0.7 * MILLIMETRE));
            List<Surroundings.Squeeze> onTheWheel = new ArrayList<>(), onTheHood = new ArrayList<>();
            for (Surroundings.Squeeze squeeze :
                    Valid.value(Surroundings.of(FIELD, List.of(launcher))).squeezing(centre, POLLEN)) {
                (squeeze.id().startsWith("Flywheel") ? onTheWheel : onTheHood).add(squeeze);
            }
            assertTrue(!onTheWheel.isEmpty() && !onTheHood.isEmpty());
            Vec3 frameAtHub = course.velocity().plus(frameSpin.cross(hub.minus(placed.origin())));

            Vec3 rolling = frameAtHub.plus(wheelSpin.cross(centre.minus(hub)));
            assertNotRubbed("rolling with the wheel at " + degrees, rolling, wheelSpin, onTheWheel);
            Vec3 carried = course.velocity().plus(frameSpin.cross(centre.minus(placed.origin())));
            assertNotRubbed("carried with the frame at " + degrees, carried, frameSpin, onTheHood);
        }
    }

    private static void assertNotRubbed(String what, Vec3 velocity, Vec3 spin, List<Surroundings.Squeeze> squeezes) {
        Bodies.Step step = Valid.value(Flight.stepping());
        for (Surroundings.Squeeze squeeze : squeezes) {
            CompliantContact.Push pushed = CompliantContact.push(BALL, velocity, spin, List.of(squeeze), step);
            Vec3 n = squeeze.normal();
            double pushing = pushed.acceleration().dot(n);
            assertTrue(what + ", " + squeeze.id() + ": pushed at " + pushed.acceleration(), pushing > 0);
            assertVec(what + ", " + squeeze.id() + ": rubbed", n.times(pushing), pushed.acceleration(), 1e-9 * pushing);
            assertVec(what + ", " + squeeze.id() + ": turned", Vec3.zero(), pushed.spinRate(), 1e-9 * pushing / R);
        }
    }

    private static List<PartContract.Probe> probes(Launcher launcher) {
        return probes(launcher, List.of(0.0, Shot.TOUCHING_IN));
    }

    private static List<PartContract.Probe> probes(Launcher launcher, List<Double> withins) {
        Launcher.Placed placed = launcher.where();
        List<PartContract.Probe> probes = new ArrayList<>();
        for (double degrees = -115; degrees <= 60; degrees += 6.9) {
            for (double out = 30; out <= 140; out += 9.1) {
                for (double across : new double[] {-0.9, -0.21, 0, 0.45}) {
                    Vec3 local = onThePath(degrees, out * MILLIMETRE).plus(new Vec3(0, across, 0));
                    for (double within : withins) {
                        probes.add(PartContract.Probe.of(placedAt(placed, local), R, within));
                    }
                }
            }
        }
        for (double degrees = HOOD_FROM - 3; degrees <= HOOD_FROM + 3; degrees += 0.37) {
            for (double out : new double[] {72, 74.6, 75.03, 75.4, 77}) {
                probes.add(PartContract.Probe.of(placedAt(placed, onThePath(degrees, out * MILLIMETRE)), R, 0));
            }
        }
        for (double degrees = EXTENSION_FROM - 4; degrees <= EXTENSION_FROM + 4; degrees += 0.53) {
            for (double out : new double[] {73.9, 75.03, 76.2}) {
                probes.add(PartContract.Probe.of(
                        placedAt(placed, onThePath(degrees, out * MILLIMETRE)), R, Shot.TOUCHING_IN));
            }
        }
        for (double degrees = Launcher.HOOD_TO_DEGREES - 3; degrees <= Launcher.HOOD_TO_DEGREES + 8; degrees += 0.7) {
            probes.add(PartContract.Probe.of(placedAt(placed, onThePath(degrees, 75.03 * MILLIMETRE)), R, 0));
        }
        return probes;
    }

    private static List<Double> contractTimes() {
        return List.of(0.096, BEGINS, 0.1013, 0.1025, ENDS, 0.109);
    }

    private static void assertKept(String what, Launcher launcher, boolean frameStill) {
        PartContract.assertKept(
                what,
                launcher,
                probes(launcher.at(Valid.time(0.1025))),
                contractTimes(),
                (seconds, touch) -> frameStill);
    }

    @Test
    public void itKeepsThePartContractStillWithItsWheelStoppedSpinningAndReversed() {
        Launcher.Course still = course(frame(10, -20, 0.4), frame(10, -20, 0.4));
        assertKept("still, stopped", launcher(still, wheel(0)), true);
        assertKept("still, spinning", launcher(still, wheel(CLOSE_LAUNCH)), true);
        assertKept("still, reversed", launcher(still, wheel(-Flywheel.TOP_TICKS_PER_SECOND, 18)), true);
    }

    @Test
    public void itKeepsThePartContractSlidingAndTurningWithItsWheelStoppedSpinningAndReversed() {
        Launcher.Frame from = frame(10, -20, 0.4);
        assertKept("sliding", launcher(course(from, frame(10.5, -20.3, 0.4)), wheel(0)), false);
        assertKept("turning", launcher(course(from, frame(10, -20, 0.55)), wheel(CLOSE_LAUNCH)), false);
        assertKept("sliding and turning", launcher(course(from, frame(9.4, -19.7, 0.3)), wheel(-CLOSE_LAUNCH)), false);
        assertKept(
                "crossing half a turn",
                launcher(
                        course(frame(-40, 3, StrictMath.PI - 0.05), frame(-40.2, 3.1, -StrictMath.PI + 0.06)),
                        wheel(Flywheel.TOP_TICKS_PER_SECOND, 18)),
                false);
        assertKept(
                "a course of no time",
                Launcher.placed(
                        faces(),
                        Valid.value(Launcher.Course.of(from, Valid.time(BEGINS), frame(3, 2, 1), Valid.time(BEGINS))),
                        wheel(CLOSE_LAUNCH)),
                true);
    }

    @Test
    public void placedAtAnyTimeItsWheelTurnsAsItWasGivenAndItsFrameMovesAsItsCourseSays() {
        Launcher.Course course = course(frame(1, 2, 0.1), frame(1.4, 2.2, 0.2));
        Launcher launcher = launcher(course, wheel(CLOSE_LAUNCH));
        for (double t : new double[] {0, BEGINS, 0.103, ENDS, 0.5, 30}) {
            Launcher then = launcher.at(Valid.time(t));
            assertEquals(course.sweep(), then.sweep(), 0);
            assertTrue(then.moving());
            Launcher.Placed placed = course.at(Valid.time(t));
            assertEquals(placed.origin(), then.where().origin());
            assertEquals(placed.yaw(), then.where().yaw(), 0);
            assertEquals(
                    launcher.at(Valid.time(t)).where(),
                    launcher.at(Valid.time(7)).at(Valid.time(t)).where());
        }
        assertTrue(
                launcher(course(frame(1, 2, 0.1), frame(1, 2, 0.1)), wheel(1)).moving());
        assertEquals(
                0,
                launcher(course(frame(1, 2, 0.1), frame(1, 2, 0.1)), wheel(1)).sweep(),
                0);
        assertEquals(
                course.sweep(),
                StrictMath.sqrt(course.velocity().dot(course.velocity()))
                        + StrictMath.abs(course.turning()) * Launcher.FARTHEST_IN,
                0);
    }

    @Test
    public void itReachesWhereSomeFaceIsWithinTheDistanceAskedAndNowhereElse() {
        Launcher launcher = launcher(course(frame(-3, 4, 2), frame(-3, 4, 2)), wheel(CLOSE_LAUNCH));
        Launcher.Placed placed = launcher.where();
        int reached = 0;
        for (double degrees = -180; degrees < 180; degrees += 11.3) {
            for (double out = 0; out <= 200; out += 7.7) {
                for (double across : new double[] {-2.3, -1.1, -0.4, 0, 0.7, 1.9}) {
                    Vec3 centre = placedAt(
                            placed, onThePath(degrees, out * MILLIMETRE).plus(new Vec3(0, across, 0)));
                    for (double by : new double[] {0.05, 0.3, R, R + Shot.MOST_TRAVEL_IN}) {
                        boolean faceWithin =
                                !launcher.squeezing(centre, Valid.length(by)).isEmpty()
                                        || !launcher.near(centre, Valid.length(1e-6), by - 1e-6)
                                                .isEmpty();
                        assertEquals(
                                "at " + degrees + " degrees, " + out + " mm out, " + across + " in across, by " + by,
                                faceWithin,
                                launcher.reaches(centre, by));
                        reached += faceWithin ? 1 : 0;
                    }
                }
            }
        }
        assertTrue(reached > 100);
        Vec3 hub = placedAt(placed, HUB);
        for (double x : new double[] {-1, 0, 1}) {
            for (double y : new double[] {-1, 0, 1}) {
                for (double z : new double[] {-1, 0, 1}) {
                    Vec3 away = new Vec3(x, y, z);
                    if (away.dot(away) > 0) {
                        Vec3 far = hub.plus(away.times(20 / StrictMath.sqrt(away.dot(away))));
                        assertTrue(
                                "20 in from the hub toward " + away, !launcher.reaches(far, R + Shot.MOST_TRAVEL_IN));
                    }
                }
            }
        }
    }

    private static final class SqueezingNothing implements Surroundings.Part {
        private final Surroundings.Part inner;

        SqueezingNothing(Surroundings.Part inner) {
            this.inner = inner;
        }

        @Override
        public List<String> ids() {
            return inner.ids();
        }

        @Override
        public boolean reaches(Vec3 centre, double by) {
            return inner.reaches(centre, by);
        }

        @Override
        public double sweep() {
            return inner.sweep();
        }

        @Override
        public boolean moving() {
            return inner.moving();
        }

        @Override
        public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
            return inner.near(centre, radius, within);
        }

        @Override
        public boolean squeezes() {
            return false;
        }

        @Override
        public List<Surroundings.Overlap> squeezing(Vec3 centre, Length radius) {
            return List.of();
        }

        @Override
        public Surroundings.Part at(GridTime time) {
            return new SqueezingNothing(inner.at(time));
        }
    }

    private static Bodies flying(Surroundings.Part launcher, Vec3 from, Vec3 velocity) {
        Bodies bodies = Bodies.unwatched(
                Valid.value(Surroundings.of(FIELD, List.of(launcher))),
                Valid.value(ShotSetup.air(ShotSettings.defaults())),
                Valid.value(ShotSetup.surfaces(ShotSettings.defaults())),
                Valid.value(Flight.stepping()));
        Valid.value(bodies.add(
                Valid.value(Bodies.Name.of("ball")),
                BALL,
                Valid.value(Bodies.Motion.of(from, velocity, Vec3.zero())),
                new Bodies.Start.Awake(Bodies.Sleep.WHEN_RESTING)));
        return bodies;
    }

    @Test
    public void aBallElsewhereOnTheFieldMovesAndCostsExactlyAsItWouldWereTheLauncherToSqueezeNothing() {
        Launcher launcher = launcher(course(frame(0, 0, 0), frame(0.2, 0.1, 0.03)), wheel(CLOSE_LAUNCH));
        Vec3 from = new Vec3(-10, 30, 20), velocity = new Vec3(40, -60, 10);
        Bodies among = flying(launcher, from, velocity);
        Bodies amongNothing = flying(new SqueezingNothing(launcher), from, velocity);
        Bodies near = flying(launcher, launcher.entry(POLLEN), launcher.feed(POLLEN));
        Bodies nearNothing = flying(new SqueezingNothing(launcher), launcher.entry(POLLEN), launcher.feed(POLLEN));
        for (int i = 0; i < 300; i++) {
            among.step();
            amongNothing.step();
            near.step();
            nearNothing.step();
        }

        assertEquals(amongNothing.body("ball"), among.body("ball"));
        assertEquals(amongNothing.work(), among.work());
        assertTrue(
                "a ball within its reach costs " + near.work() + " against " + nearNothing.work(),
                near.work() > nearNothing.work());
    }

    @Test
    public void forEveryFrameAndWheelTheTypesAdmitWhatSqueezesOrTouchesABallWithinItsReachMovesAtAFiniteVelocity() {
        double most = Launcher.Frame.MOST_FROM_CENTRE_IN;
        double fastest = 0.999 * Flight.MOST_SPEED_IN_PER_S * (ENDS - BEGINS);
        double fastestTurn = 0.999 * Flight.MOST_SPEED_IN_PER_S / Launcher.FARTHEST_IN * (ENDS - BEGINS);
        List<Flywheel.AngularSpeed> wheels = List.of(
                wheel(Flywheel.TOP_TICKS_PER_SECOND, 18),
                wheel(-Flywheel.TOP_TICKS_PER_SECOND, 18),
                wheel(Double.NaN, 18),
                wheel(1, 10000));
        int met = 0;
        for (double x : new double[] {-most, 0, most}) {
            for (double y : new double[] {-most, most}) {
                for (double yaw : new double[] {-StrictMath.PI, -1, 3, StrictMath.PI}) {
                    Launcher.Frame from = frame(x, y, yaw);
                    List<Launcher.Course> courses = List.of(
                            course(from, from),
                            course(from, frame(x - StrictMath.signum(x) * fastest, y, yaw)),
                            course(from, frame(x, y, yaw + fastestTurn)),
                            course(from, frame(x - StrictMath.signum(x) * fastest / 2, y, yaw - fastestTurn / 2)));
                    for (Launcher.Course course : courses) {
                        for (Flywheel.AngularSpeed spinning : wheels) {
                            for (double t : new double[] {-250, BEGINS, ENDS, 250}) {
                                Launcher then = launcher(course, spinning).at(Valid.time(t));
                                Surroundings around = Valid.value(Surroundings.of(FIELD, List.of(then)));
                                for (double degrees = -75; degrees <= 25; degrees += 25) {
                                    Vec3 centre = placedAt(then.where(), onThePath(degrees, wheelRing(0) + R));
                                    assertTrue(then.reaches(centre, R));
                                    List<Surroundings.Overlap> overlaps = then.squeezing(centre, POLLEN);
                                    assertTrue("squeezed at " + centre, !overlaps.isEmpty());
                                    assertEquals(
                                            overlaps.size(),
                                            around.squeezing(centre, POLLEN).size());
                                    for (Surroundings.Squeeze squeeze : around.squeezing(centre, POLLEN)) {
                                        assertTrue(squeeze.velocity().isFinite()
                                                && squeeze.spin().isFinite());
                                        met++;
                                    }
                                }
                                Vec3 entry = then.entry(POLLEN);
                                assertEquals(
                                        1,
                                        then.near(entry, POLLEN, Shot.TOUCHING_IN)
                                                .size());
                                assertTrue(then.feed(POLLEN).isFinite());
                            }
                        }
                    }
                }
            }
        }
        assertTrue(met > 1000);
    }
}
