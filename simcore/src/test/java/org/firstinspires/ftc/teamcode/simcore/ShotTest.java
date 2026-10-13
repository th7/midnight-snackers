package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.Test;

public class ShotTest {
    private static final double G = Flight.GRAVITY_IN_PER_S2;
    private static final Pollen POLLEN = Valid.value(Pollen.of(2.8, 24.9, 0.634));
    private static final double R = POLLEN.radius();
    private static final double ALPHA = POLLEN.inertiaFactor();
    private static final Field BARE = Valid.value(Field.of(
            Valid.length(141.17),
            Valid.length(11.64),
            List.of(),
            List.of(),
            List.of(),
            List.of(),
            List.of(),
            List.of()));
    private static final Colliders WALLS = Valid.value(Colliders.of(BARE, List.of()));

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    private static Bounce bounce(double restitution, double friction, double grip) {
        return Valid.value(Bounce.of(restitution, friction, grip));
    }

    private static Surfaces surfaces(Bounce floor, double rollingResistance) {
        Bounce other = bounce(0.5, 0.3, 0.15);
        return Valid.value(Surfaces.of(floor, other, other, other, other, other, 0, 4, 0, 0, rollingResistance));
    }

    private static Surfaces floorOf(double restitution, double friction, double grip) {
        return surfaces(bounce(restitution, friction, grip), 0);
    }

    private static Surfaces realistic() {
        Bounce other = bounce(0.5, 0.3, 0.15);
        return Valid.value(
                Surfaces.of(bounce(0.55, 0.5, 0.2), other, other, other, other, other, 0.038, 4, 0.1, 0.06, 0.04));
    }

    private static Air air() {
        return Valid.value(Air.of(1.204, Vec3.zero(), 0.45, 0.15, 2, 0.4, 0.012, 0.5));
    }

    private static World world(Air air, Surfaces surfaces, double step, double longest, List<World.Piece> pieces) {
        return Valid.value(World.of(WALLS, air, surfaces, seconds(step), seconds(longest), pieces));
    }

    private static World vacuum(Surfaces surfaces, double longest) {
        return world(Air.vacuum(), surfaces, 1e-4, longest, List.of());
    }

    private static WheelPair.Launched launched(Vec3 at, Vec3 velocity, Vec3 spin) {
        return new WheelPair.Launched(at, velocity, spin, 0, 0, 0, 0);
    }

    private static Shot.Finished flown(World world, Vec3 at, Vec3 velocity, Vec3 spin) {
        return finished(Valid.value(Shot.launch(world, POLLEN, launched(at, velocity, spin))), 100);
    }

    private static Shot.Finished finished(Shot.Flying flying, double until) {
        Shot shot = flying.advance(seconds(until));
        assertTrue("finished by " + until + " s", shot instanceof Shot.Finished);
        return (Shot.Finished) shot;
    }

    private static List<Shot.Frame> framesBeforeTheFirstBounce(Shot shot) {
        List<Shot.Frame> before = new ArrayList<>();
        double first = shot.story().impacts().isEmpty()
                ? Double.MAX_VALUE
                : shot.story().impacts().get(0).t().value();
        for (Shot.Frame frame : shot.story().frames()) {
            if (frame.t().value() < first) {
                before.add(frame);
            }
        }
        return before;
    }

    private static void assertVec(String what, Vec3 expected, Vec3 actual, double delta) {
        assertEquals(what + " x of " + actual, expected.x(), actual.x(), delta);
        assertEquals(what + " y of " + actual, expected.y(), actual.y(), delta);
        assertEquals(what + " z of " + actual, expected.z(), actual.z(), delta);
    }

    @Test
    public void inAVacuumAPollenFliesTheParabola() {
        Vec3 from = new Vec3(-40, 0, 30), velocity = new Vec3(120, 30, 250);
        Shot.Finished shot = flown(vacuum(floorOf(0.5, 0.5, 0.2), 0.5), from, velocity, Vec3.zero());

        List<Shot.Frame> flying = framesBeforeTheFirstBounce(shot);
        assertTrue(flying.size() > 100);
        for (Shot.Frame frame : flying) {
            double t = frame.t().value();
            assertVec(
                    "at " + t,
                    new Vec3(
                            from.x() + velocity.x() * t,
                            from.y() + velocity.y() * t,
                            from.z() + velocity.z() * t - G * t * t / 2),
                    frame.pollen().at(),
                    1e-9);
        }
    }

    @Test
    public void itMeetsTheFloorWhenAndWhereTheParabolaSays() {
        Vec3 from = new Vec3(-40, 0, 30), velocity = new Vec3(40, 10, 250);
        Shot.Finished shot = flown(vacuum(floorOf(0.5, 0.5, 0.2), 3), from, velocity, Vec3.zero());

        double drop = from.z() - R;
        double t = (velocity.z() + Math.sqrt(velocity.z() * velocity.z() + 2 * G * drop)) / G;
        Shot.Impact first = shot.story().impacts().get(0);
        assertEquals(Surface.FLOOR, first.hit().surface());
        assertEquals(t, first.t().value(), 1e-9);
        assertVec(
                "where it lands",
                new Vec3(from.x() + velocity.x() * t, from.y() + velocity.y() * t, 0),
                first.at(),
                1e-7);
        assertEquals(velocity.z() - G * t, -first.hit().approach(), 1e-6);
    }

    @Test
    public void aPollenDroppedThroughStillAirFallsAsQuadraticDragSays() {
        Air still = Valid.value(Air.of(1.204, Vec3.zero(), 0.45, 0, 2, 0.4, 0, 0));
        double volume = POLLEN.volumeCubicMetres(), mass = POLLEN.kilograms();
        double sinking = G * (mass - 1.204 * volume) / mass;
        double k = 1.204 * POLLEN.areaSquareMetres() / (2 * mass) * Length.METRES_PER_INCH * 0.45;
        double terminal = Math.sqrt(sinking / k);
        double from = 400;
        Shot.Finished shot = flown(
                world(still, floorOf(0.5, 0.5, 0.2), 1e-4, 0.8, List.of()),
                new Vec3(0, 0, from),
                Vec3.zero(),
                Vec3.zero());

        for (Shot.Frame frame : framesBeforeTheFirstBounce(shot)) {
            double t = frame.t().value();
            double z = from - terminal * terminal / sinking * Math.log(Math.cosh(sinking * t / terminal));
            assertEquals("height at " + t, z, frame.pollen().at().z(), 1e-6);
            assertEquals(
                    "speed at " + t,
                    -terminal * Math.tanh(sinking * t / terminal),
                    frame.pollen().velocity().z(),
                    1e-6);
        }
    }

    @Test
    public void aDroppedPollenBouncesLowerEachTimeAndRestsOnceItCanNoLongerBounce() {
        double e = 0.6, height = 20, threshold = 4;
        Shot.Finished shot =
                flown(vacuum(floorOf(e, 0.5, 0.2), 10), new Vec3(0, 0, R + height), Vec3.zero(), Vec3.zero());

        double falling = Math.sqrt(2 * height / G), landing = G * falling;
        double t = falling;
        int bounces = 0;
        for (double approach = landing; approach > threshold; approach *= e) {
            Shot.Impact impact = shot.story().impacts().get(bounces);
            assertEquals("bounce " + bounces + " lands at", t, impact.t().value(), 1e-7);
            assertEquals(
                    "bounce " + bounces + " comes in at", approach, impact.hit().approach(), 1e-6);
            assertEquals("bounce " + bounces + " restitution", e, impact.hit().restitution(), 1e-12);
            t += 2 * approach * e / G;
            bounces++;
        }
        assertEquals(bounces, shot.story().impacts().size());
        Shot.Finish finish = shot.finish();
        assertTrue(finish.toString(), finish instanceof Shot.Finish.AtRest);
        Shot.Finish.AtRest rest = (Shot.Finish.AtRest) finish;
        assertTrue(rest.where() instanceof Shot.Resting.OnTheFloor);
        assertEquals(t + Shot.REST_SECONDS, rest.t().value(), 2e-4);
        assertEquals(R, rest.at().z(), 1e-6);
    }

    @Test
    public void aSlidingPollenRollsOnAtTheSpeedItsAngularMomentumAboutTheFloorAllows() {
        Vec3 from = new Vec3(-60, 0, R);
        double v0 = 150;
        Shot.Flying flying = Valid.value(Shot.launch(
                vacuum(floorOf(0.5, 0.5, 0.2), 0.6), POLLEN, launched(from, new Vec3(v0, 0, 0), Vec3.zero())));
        Shot shot = flying.advance(seconds(0.6));

        Shot.Frame last = shot.story().frames().get(shot.story().frames().size() - 1);
        double rolling = v0 / (1 + ALPHA);
        assertEquals(rolling, last.pollen().velocity().x(), 1e-9);
        assertEquals(rolling, last.pollen().spin().y() * R, 1e-9);
        assertEquals(Shot.Phase.ROLLING, last.pollen().phase());
        assertTrue(shot.story().impacts().isEmpty());
    }

    @Test
    public void aRollingPollenSlowsAtItsRollingResistanceTimesGravity() {
        double resistance = 0.04, v0 = 100;
        World world = world(Air.vacuum(), surfaces(bounce(0.55, 0.5, 0.2), resistance), 1e-4, 1, List.of());
        Shot shot = Valid.value(Shot.launch(
                        world, POLLEN, launched(new Vec3(-60, 0, R), new Vec3(v0, 0, 0), new Vec3(0, v0 / R, 0))))
                .advance(seconds(1));

        Shot.Frame last = shot.story().frames().get(shot.story().frames().size() - 1);
        assertEquals(
                v0 - resistance * G * last.t().value(), last.pollen().velocity().x(), 1e-2);
    }

    @Test
    public void backspinKicksABounceUpAndTopspinMakesItSkid() {
        double spin = 60;
        double none = leavingAngle(Vec3.zero()),
                back = leavingAngle(new Vec3(0, -spin, 0)),
                top = leavingAngle(new Vec3(0, spin, 0));

        assertTrue("backspin " + back + " leaves steeper than no spin " + none, back > none + 1);
        assertTrue("topspin " + top + " leaves flatter than no spin " + none, top < none - 1);
    }

    private static double leavingAngle(Vec3 spin) {
        Shot.Finished shot =
                flown(vacuum(floorOf(0.6, 0.5, 0.15), 0.3), new Vec3(-20, 0, 12), new Vec3(170, 0, -170), spin);
        Vec3 after = shot.story().impacts().get(0).velocityAfter();
        return Math.toDegrees(Math.atan2(after.z(), after.x()));
    }

    @Test
    public void aPollenNeverEndsAPieceOfTimeInsideAnything() {
        Shot.Finished shot = flown(
                world(air(), realistic(), 1e-4, 6, List.of()),
                new Vec3(50, 50, 5),
                new Vec3(300, 280, -50),
                new Vec3(100, -80, 200));

        assertTrue(shot.finish().toString(), shot.finish() instanceof Shot.Finish.AtRest);
        assertTrue(shot.story().impacts().toString(), shot.story().impacts().size() > 5);
        for (Shot.Frame frame : shot.story().frames()) {
            for (Collider collider : WALLS.all()) {
                double gap = WALLS.gap(collider, frame.pollen().at(), Valid.length(R));
                assertTrue(collider.name() + " at " + frame.t() + ": " + gap, gap > -1e-6);
            }
        }
    }

    @Test
    public void aShotIsTheSameEveryTimeHoweverItIsAdvanced() {
        World world = world(air(), floorOf(0.55, 0.5, 0.2), 1e-4, 3, List.of());
        WheelPair.Launched launch = launched(new Vec3(50, 50, 5), new Vec3(300, 280, -50), new Vec3(100, -80, 200));
        Shot once = Valid.value(Shot.launch(world, POLLEN, launch)).advance(seconds(3));
        Shot tenths = inChunks(Valid.value(Shot.launch(world, POLLEN, launch)), 0.1);
        Shot odd = inChunks(Valid.value(Shot.launch(world, POLLEN, launch)), 0.037);

        assertSameFrames(once, tenths);
        assertSameFrames(once, odd);
        assertSameFrames(once, Valid.value(Shot.launch(world, POLLEN, launch)).advance(seconds(3)));
    }

    private static Shot inChunks(Shot.Flying flying, double chunk) {
        Shot shot = flying;
        double until = 0;
        while (shot instanceof Shot.Flying still && until < 3) {
            until += chunk;
            shot = still.advance(seconds(Math.min(until, 3)));
        }
        return shot;
    }

    private static void assertSameFrames(Shot expected, Shot actual) {
        List<Shot.Frame> a = expected.story().frames(), b = actual.story().frames();
        assertEquals(a.size(), b.size());
        for (int i = 0; i < a.size(); i++) {
            Shot.Moving x = a.get(i).pollen(), y = b.get(i).pollen();
            assertEquals(
                    Double.doubleToRawLongBits(a.get(i).t().value()),
                    Double.doubleToRawLongBits(b.get(i).t().value()));
            assertEquals("frame " + i, bits(x.at()), bits(y.at()));
            assertEquals("frame " + i, bits(x.velocity()), bits(y.velocity()));
            assertEquals("frame " + i, bits(x.spin()), bits(y.spin()));
        }
    }

    private static List<Long> bits(Vec3 v) {
        return List.of(
                Double.doubleToRawLongBits(v.x()),
                Double.doubleToRawLongBits(v.y()),
                Double.doubleToRawLongBits(v.z()));
    }

    @Test
    public void flightToTheFirstBounceConvergesAsTheStepShrinks() {
        WheelPair.Launched launch = launched(new Vec3(-50, 10, 14), new Vec3(60, -10, 240), new Vec3(10, -60, 25));
        Shot.Impact coarse = flown(
                        world(air(), floorOf(0.55, 0.5, 0.2), 1e-4, 2, List.of()),
                        launch.at(),
                        launch.velocity(),
                        launch.spin())
                .story()
                .impacts()
                .get(0);
        Shot.Impact fine = flown(
                        world(air(), floorOf(0.55, 0.5, 0.2), 5e-5, 2, List.of()),
                        launch.at(),
                        launch.velocity(),
                        launch.spin())
                .story()
                .impacts()
                .get(0);

        assertVec("first bounce", coarse.at(), fine.at(), 1e-6);
        assertEquals(coarse.t().value(), fine.t().value(), 1e-9);
    }

    @Test
    public void aShotStillMovingWhenItsTimeRunsOutSaysSo() {
        Shot.Finished shot =
                flown(vacuum(floorOf(0.5, 0.5, 0.2), 0.2), new Vec3(0, 0, 20), new Vec3(0, 0, 300), Vec3.zero());

        assertTrue(shot.finish() instanceof Shot.Finish.OutOfTime);
        assertEquals(0.2, ((Shot.Finish.OutOfTime) shot.finish()).t().value(), 1e-4 + 1e-12);
    }

    @Test
    public void aPollenThatClearsAWallLeavesTheFieldOverIt() {
        Shot.Finished far =
                flown(vacuum(floorOf(0.5, 0.5, 0.2), 3), new Vec3(60, 0, 20), new Vec3(200, 0, 150), Vec3.zero());
        Shot.Finished red =
                flown(vacuum(floorOf(0.5, 0.5, 0.2), 3), new Vec3(0, 60, 20), new Vec3(0, 200, 150), Vec3.zero());

        assertEquals(Colliders.Side.FAR, ((Shot.Finish.LeftTheField) far.finish()).side());
        assertEquals(Colliders.Side.RED, ((Shot.Finish.LeftTheField) red.finish()).side());
    }

    @Test
    public void tooMuchWorkIsABudgetForEachSimulatedSecond() {
        assertFalse(Shot.tooMuchWork(Shot.MOST_WORK_PER_SECOND, seconds(1)));
        assertTrue(Shot.tooMuchWork(Shot.MOST_WORK_PER_SECOND + 1, seconds(1)));
        assertFalse(Shot.tooMuchWork(Shot.MOST_WORK_PER_SECOND / 10, seconds(0)));
        assertTrue(Shot.tooMuchWork(Shot.MOST_WORK_PER_SECOND / 10 + 1, seconds(0)));
    }

    @Test
    public void framesComeEveryFourMillisecondsAndEachBounceTwice() {
        Shot.Finished shot =
                flown(vacuum(floorOf(0.6, 0.5, 0.2), 10), new Vec3(0, 0, R + 20), Vec3.zero(), Vec3.zero());
        List<Shot.Frame> frames = shot.story().frames();

        assertEquals(0, frames.get(0).t().value(), 0);
        assertVec("first frame", new Vec3(0, 0, R + 20), frames.get(0).pollen().at(), 0);
        int paired = 0;
        for (int i = 1; i < frames.size(); i++) {
            double gap = frames.get(i).t().value() - frames.get(i - 1).t().value();
            assertTrue("frame " + i + " after " + gap + " s", gap >= 0 && gap <= Shot.FRAME_SECONDS + 1e-4 + 1e-12);
            if (gap == 0) {
                paired++;
            }
        }
        assertEquals(shot.story().impacts().size(), paired);
    }

    @Test
    public void aPollenRollingIntoASleepingPieceWakesItAndSetsItRolling() {
        Pollen nectar = Valid.value(Pollen.of(3.6, 41.3, 0.634));
        World.Piece resting = new World.Piece(
                "Blue Nectar", Field.Kind.NECTAR, Optional.of("Blue"), nectar, new Vec3(0, 0, 1.8), false);
        World world = world(Air.vacuum(), floorOf(0.55, 0.5, 0.2), 1e-4, 1.2, List.of(resting));
        Shot shot = Valid.value(Shot.launch(
                        world, POLLEN, launched(new Vec3(-30, 0, R), new Vec3(100, 0, 0), new Vec3(0, 100 / R, 0))))
                .advance(seconds(1.2));

        Shot.Track track = shot.story().pieces().get(0);
        assertEquals("Blue Nectar", track.piece().name());
        assertTrue(track.samples().size() > 10);
        Shot.Placed lastSeen = track.samples().get(track.samples().size() - 1);
        assertTrue("the nectar rolled on to " + lastSeen.at(), lastSeen.at().x() > 5);
        assertTrue(
                shot.story().impacts().stream().anyMatch(impact -> impact.hit().surface() == Surface.PIECE));
    }

    @Test
    public void aPollenThatWouldStartInsideSomethingIsRefusedByName() {
        String said = Shot.launch(
                        vacuum(floorOf(0.5, 0.5, 0.2), 1),
                        POLLEN,
                        launched(new Vec3(0, 0, 1), Vec3.zero(), Vec3.zero()))
                .fold(shot -> "launched", rule -> rule);

        assertTrue(said, said.contains("Floor"));
    }

    @Test
    public void aSpinningPollenTurnsByItsSpin() {
        Shot.Finished shot =
                flown(vacuum(floorOf(0.5, 0.5, 0.2), 0.1), new Vec3(0, 0, 60), Vec3.zero(), new Vec3(0, 0, 10));
        Shot.Frame last = shot.story().frames().get(shot.story().frames().size() - 1);
        Turn turned = last.pollen().turn();
        double angle = 10 * last.t().value();

        assertEquals(Math.cos(angle / 2), turned.w(), 1e-9);
        assertEquals(Math.sin(angle / 2), turned.z(), 1e-9);
    }
}
