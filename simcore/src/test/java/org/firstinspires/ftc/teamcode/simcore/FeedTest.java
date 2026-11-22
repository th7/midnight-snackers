package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class FeedTest {
    private static final double MILLIMETRE = 1 / 25.4;
    private static final double R_MM = Launcher.POLLEN_RADIUS_IN / MILLIMETRE;
    private static final double WALL_MM = 82.27;
    private static final double ARM_HALF_THICKNESS_MM = 2;
    private static final double STILL_WITHIN_IN = 1e-3;
    private static final double ROBOT_CENTRE_BACK_MM = 204;
    private static final String SILL_SOURCE = "doc/plans/spindexer-sim.md, Findings, The front sill holds: a pollen"
            + " leans out into the front window, its centre past the wall's 46.97 mm, onto the 16.12 mm sill's edge,"
            + " which holds it at 52.6 mm while at rest";
    private static final String NOTCH_SOURCE = "doc/plans/spindexer-sim.md, Questions for the team, 4: a lifted"
            + " pollen's top pokes up into the top plate's notch, which holds it down when the wheel does not take it";
    private static final double ON_THE_FLOOR_MM = 15 + R_MM;
    private static final double OVERLAP_WITHIN_IN = 0.02;
    private static final double TILT = StrictMath.toRadians(16.46);

    private static final String CAD_SILL_SOURCE =
            "issue 180, The back sill: as drawn, at z 43.12, the wheel cannot" + " draw a pollen out over it";
    private static final String DEFAULT_FEED_SOURCE = "issue 180, the session owner's decision of 21 November: the back"
            + " sill at z 30 and the ramp up from 0.30 s to 1.20 s, so that the default feed throws both pollen";
    private static final String THROW_SOURCE = "doc/plans/runtime-flight.md, stage 4, Tests first: the launcher"
            + " throws at 70 +- 3 degrees, with backspin, antiparallel to the wheel's, with 0.9 <= S <= 1.2, where S,"
            + " the spin ratio, is |w| r / |v|";
    private static final double THROWN_AT_DEGREES = 70;
    private static final double THROWN_WITHIN_DEGREES = 3;
    private static final double LEAST_SPIN_RATIO = 0.9;
    private static final double MOST_SPIN_RATIO = 1.2;

    private static Feed.RampSchedule heldUp() {
        return Valid.value(Feed.RampSchedule.of(
                Valid.value(Spindexer.RampAngle.ofDegrees(0)),
                Valid.value(Spindexer.RampAngle.ofDegrees(35)),
                Valid.value(Seconds.of(0)),
                Valid.value(Seconds.of(10))));
    }

    private static void assertThrownAsStageFourThrows(Feed.Event.Thrown thrown) {
        Vec3 v = thrown.velocity();
        double degreesUp = StrictMath.toDegrees(StrictMath.atan2(v.z(), StrictMath.hypot(v.x(), v.y())));
        double speed = StrictMath.sqrt(v.dot(v));
        double spinRatio = StrictMath.sqrt(thrown.spin().dot(thrown.spin())) * Launcher.POLLEN_RADIUS_IN / speed;
        assertEquals(THROW_SOURCE + ": " + thrown, THROWN_AT_DEGREES, degreesUp, THROWN_WITHIN_DEGREES);
        assertTrue(
                THROW_SOURCE + ": about the axle " + thrown.spin(),
                thrown.spin().y() < 0);
        assertTrue(
                THROW_SOURCE + ": a spin ratio of " + spinRatio,
                spinRatio >= LEAST_SPIN_RATIO && spinRatio <= MOST_SPIN_RATIO);
    }

    private static Vec3 spindexerMm(Vec3 world) {
        double cos = StrictMath.cos(TILT), sin = StrictMath.sin(TILT);
        double back = ROBOT_CENTRE_BACK_MM - world.x() / MILLIMETRE - 173.96;
        double up = world.z() / MILLIMETRE - 28.78;
        return new Vec3(-world.y() / MILLIMETRE, -back * cos - up * sin, -back * sin + up * cos);
    }

    private static double degreesFromTheBack(Vec3 world) {
        Vec3 spindexer = spindexerMm(world);
        return StrictMath.toDegrees(StrictMath.atan2(-spindexer.x(), -spindexer.y()));
    }

    private static Feed.RampSchedule raisedThenLoweredAt(double seconds) {
        return Valid.value(Feed.RampSchedule.of(
                Valid.value(Spindexer.RampAngle.ofDegrees(0)),
                Valid.value(Spindexer.RampAngle.ofDegrees(35)),
                Valid.value(Seconds.of(0)),
                Valid.value(Seconds.of(seconds))));
    }

    private static Feed.RampSchedule neverRaised() {
        return Valid.value(Feed.RampSchedule.of(
                Valid.value(Spindexer.RampAngle.ofDegrees(0)),
                Valid.value(Spindexer.RampAngle.ofDegrees(35)),
                Valid.value(Seconds.of(9)),
                Valid.value(Seconds.of(10))));
    }

    private static Feed.Pollen at(double degreesFromTheBack) {
        return Valid.value(Feed.Pollen.againstTheWall(degreesFromTheBack));
    }

    private static double turnedDegrees(Feed.Fed fed) {
        double turned = 0;
        double last = degreesFromTheBack(fed.frames().get(0).pollen().get(0).at());
        for (Feed.Frame frame : fed.frames()) {
            assertEquals(frame.t() + ": " + fed.events(), 1, frame.pollen().size());
            double now = degreesFromTheBack(frame.pollen().get(0).at());
            turned += StrictMath.IEEEremainder(now - last, 360);
            last = now;
        }
        return turned;
    }

    private static double againstTheFrontArmDegrees() {
        double centre = WALL_MM - R_MM;
        return 180 - StrictMath.toDegrees(StrictMath.asin((R_MM + ARM_HALF_THICKNESS_MM) / centre));
    }

    private static Feed.Plan plan() {
        return Valid.value(Feed.Plan.defaults());
    }

    private static Feed.Fed fed(Checked<Feed.Plan> plan) {
        return Valid.value(Valid.value(plan).run());
    }

    @Test
    public void withTheSpinnerStoppedAPollenAtRestAgainstTheWallAndAnArmStaysAtRestForASecond() {
        Feed.Fed fed = fed(plan().withSpinnerPower(Power.none())
                .withPollen(List.of(Valid.value(Feed.Pollen.againstTheWall(againstTheFrontArmDegrees()))))
                .withRamp(Valid.value(Feed.RampSchedule.of(
                        Valid.value(Spindexer.RampAngle.ofDegrees(0)),
                        Valid.value(Spindexer.RampAngle.ofDegrees(35)),
                        Valid.value(Seconds.of(1)),
                        Valid.value(Seconds.of(1.2)))))
                .lasting(Valid.value(Seconds.of(2))));

        Vec3 start = fed.frames().get(0).pollen().get(0).at();
        int seen = 0;
        for (Feed.Frame frame : fed.frames()) {
            if (frame.t().value() < 1) {
                Vec3 off = frame.pollen().get(0).at().minus(start);
                assertTrue(frame.t() + ": " + off, StrictMath.sqrt(off.dot(off)) <= STILL_WITHIN_IN);
                seen++;
            }
        }
        assertEquals("a frame each 4 ms through the first second", 250, seen);
        assertTrue(fed.finish().toString(), fed.finish() instanceof Feed.Finish.AtRest);
    }

    @Test
    public void aTurningSpinnerCarriesAPollenRoundPastTheBackStationTwiceWithTheRampDown() {
        Feed.Fed fed =
                fed(plan().withPollen(List.of(at(90))).withRamp(neverRaised()).lasting(Valid.value(Seconds.of(2.2))));

        double turned = turnedDegrees(fed);
        assertTrue("from 90 degrees past 360 and 720, turned " + turned, turned > 630 && turned < 720);
        assertEquals(List.of(), fed.events());
        assertTrue(fed.finish().toString(), fed.finish() instanceof Feed.Finish.OutOfTime);
    }

    @Test
    public void atOneTurnASecondAPollenLeansOnTheFrontSillAndStaysInsideItForTwoTurns() {
        Feed.Fed fed =
                fed(plan().withPollen(List.of(at(90))).withRamp(neverRaised()).lasting(Valid.value(Seconds.of(2.2))));

        double farthest = 0;
        for (Feed.Frame frame : fed.frames()) {
            for (Feed.Seen pollen : frame.pollen()) {
                Vec3 spindexer = spindexerMm(pollen.at());
                double out = StrictMath.hypot(spindexer.x(), spindexer.y());
                if (StrictMath.abs(degreesFromTheBack(pollen.at())) > 150) {
                    farthest = StrictMath.max(farthest, out);
                }
            }
        }
        assertTrue(
                SILL_SOURCE + ": out to " + farthest + " mm from the axis at the front",
                farthest > 52 && farthest < WALL_MM - R_MM + 10);
        assertEquals(List.of(), fed.events());
    }

    @Test
    public void withTheRampUpAsThePollenPassesTheWheelTakesItAndItLeavesTheHoodAsStageFoursShotsDo() {
        Feed.Fed fed = fed(plan().withPollen(List.of(at(90))).withRamp(heldUp()).lasting(Valid.value(Seconds.of(2))));
        Feed.Fed down =
                fed(plan().withPollen(List.of(at(90))).withRamp(neverRaised()).lasting(Valid.value(Seconds.of(2))));

        assertEquals("with the ramp down: " + down.events(), List.of(), down.events());
        assertEquals(DEFAULT_FEED_SOURCE + ": " + fed.events(), 1, fed.events().size());
        assertTrue(fed.events().toString(), fed.events().get(0) instanceof Feed.Event.Thrown);
        assertThrownAsStageFourThrows((Feed.Event.Thrown) fed.events().get(0));
        assertTrue(fed.finish().toString(), fed.finish() instanceof Feed.Finish.AllLeft);
    }

    @Test
    public void withTheRampUpAndTheFlywheelStoppedAPollenIsLiftedStaysInAndFallsBackToTheFloorAsTheRampLowers() {
        Feed.Fed fed = fed(plan().withFlywheel(Flywheel.AngularSpeed.stopped())
                .withPollen(List.of(at(90)))
                .withRamp(raisedThenLoweredAt(1.3))
                .lasting(Valid.value(Seconds.of(2.5))));

        double highest = 0;
        Vec3 last = Vec3.zero();
        for (Feed.Frame frame : fed.frames()) {
            assertEquals(frame.t() + ": " + fed.events(), 1, frame.pollen().size());
            last = spindexerMm(frame.pollen().get(0).at());
            if (frame.t().value() < 1.3) {
                highest = StrictMath.max(highest, last.z());
            }
        }
        assertTrue(NOTCH_SOURCE + ": lifted to " + highest + " mm", highest > ON_THE_FLOOR_MM + 10);
        assertEquals("back on the floor at the end", ON_THE_FLOOR_MM, last.z(), 1);
        assertTrue(fed.finish().toString(), fed.finish() instanceof Feed.Finish.OutOfTime);
    }

    @Test
    public void aPollenHeldUnderTheNotchStallsTheSpinnerAndTheFeedReportsTheJamOnce() {
        Feed.Fed fed = fed(plan().withFlywheel(Flywheel.AngularSpeed.stopped())
                .withPollen(List.of(at(90)))
                .withRamp(raisedThenLoweredAt(1.6))
                .lasting(Valid.value(Seconds.of(1.8))));

        assertEquals(NOTCH_SOURCE + ": " + fed.events(), 1, fed.events().size());
        assertTrue(fed.events().toString(), fed.events().get(0) instanceof Feed.Event.Jammed);
        Feed.Event.Jammed jammed = (Feed.Event.Jammed) fed.events().get(0);
        assertEquals(List.of("Pollen 1"), jammed.pollen());
        assertTrue(jammed.toString(), jammed.at().value() > 1.1 && jammed.at().value() < 1.5);
    }

    @Test
    public void twoPollenWithTheRampHeldUpLeaveOneAtATimeAndNeverOverlap() {
        Feed.Fed fed = fed(
                plan().withPollen(List.of(at(90), at(270))).withRamp(heldUp()).lasting(Valid.value(Seconds.of(2.5))));

        for (Feed.Frame frame : fed.frames()) {
            if (frame.pollen().size() == 2) {
                Vec3 apart =
                        frame.pollen().get(0).at().minus(frame.pollen().get(1).at());
                assertTrue(
                        frame.t() + ": centres " + StrictMath.sqrt(apart.dot(apart)) + " in apart",
                        StrictMath.sqrt(apart.dot(apart)) >= 2 * Launcher.POLLEN_RADIUS_IN - OVERLAP_WITHIN_IN);
            }
        }
        assertEquals(fed.events().toString(), 2, fed.events().size());
        Feed.Event.Thrown first = (Feed.Event.Thrown) fed.events().get(0);
        Feed.Event.Thrown second = (Feed.Event.Thrown) fed.events().get(1);
        assertTrue(
                "half a turn apart: " + fed.events(),
                second.at().value() - first.at().value() > 0.25);
        assertTrue(fed.finish().toString(), fed.finish() instanceof Feed.Finish.AllLeft);
    }

    @Test
    public void aFeedPastItsBudgetOfWorkEndsThereAndSaysSo() {
        Feed.Fed fed = fed(Checked.ok(plan().withBudget(Valid.value(Feed.Budget.ofWorkPerSecond(100_000)))));

        assertTrue(fed.finish().toString(), fed.finish() instanceof Feed.Finish.TooMuchWork);
        assertTrue(fed.toString(), fed.work() > 100_000 * fed.simulated().value());
        assertTrue(fed.finish().toString(), fed.finish().at().value() < 3);
        assertEquals(
                fed.finish().at(), fed.frames().get(fed.frames().size() - 1).t());
    }

    @Test
    public void aBudgetIsSomeWorkASecondUpToTheEnginesOwn() {
        assertTrue(Feed.Budget.ofWorkPerSecond(0) instanceof Checked.Rejected);
        assertTrue(Feed.Budget.ofWorkPerSecond(3_000_001) instanceof Checked.Rejected);
        assertTrue(Feed.Budget.ofWorkPerSecond(3_000_000) instanceof Checked.Ok);
    }

    @Test
    public void theDefaultFeedThrowsBothPollenOneAtATimeAndTheCadsSillJamsIt() {
        Feed.Fed fed = fed(Checked.ok(plan()));
        Feed.Fed cad = fed(Checked.ok(plan().withShape(Valid.value(Spindexer.Shape.cad()))));

        assertEquals(DEFAULT_FEED_SOURCE + ": " + fed.events(), 2, fed.events().size());
        assertTrue(fed.events().toString(), fed.events().get(0) instanceof Feed.Event.Thrown);
        assertTrue(fed.events().toString(), fed.events().get(1) instanceof Feed.Event.Thrown);
        assertTrue(fed.finish().toString(), fed.finish() instanceof Feed.Finish.AllLeft);
        assertTrue(
                CAD_SILL_SOURCE + ": " + cad.events(),
                cad.events().stream().noneMatch(Feed.Event.Thrown.class::isInstance));
    }
}
