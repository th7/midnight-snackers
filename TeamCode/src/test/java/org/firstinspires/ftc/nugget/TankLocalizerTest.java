package org.firstinspires.ftc.nugget;

import static org.junit.Assert.assertEquals;

import com.acmerobotics.roadrunner.Pose2d;
import org.junit.Before;
import org.junit.Test;

public class TankLocalizerTest {
    private static final double EXACTLY = 1e-9;
    private static final double IN_PER_TICK = 0.001;
    private static final double TRACK_WIDTH_TICKS = 20000;
    private static final long LOOP_NANOS = 20_000_000L;
    private static final Pose2d START = new Pose2d(10, 20, Math.PI / 2);

    private final FakeNugget nugget = new FakeNugget();
    private TankLocalizer localizer;

    @Before
    public void built() {
        Trajectories.Params params = new Trajectories.Params();
        params.inPerTick = IN_PER_TICK;
        params.trackWidthTicks = TRACK_WIDTH_TICKS;
        new TankDrive(nugget.hardware());
        localizer = new TankLocalizer(nugget.hardware(), params, START);
    }

    private void tick() {
        nugget.nanos += LOOP_NANOS;
        localizer.loop();
    }

    @Test
    public void itIsWhereItStartedUntilASideHasGoneAnywhere() {
        tick();
        tick();

        assertEquals(START, localizer.pose());
        assertEquals(0, localizer.velocity().linearVel.x, EXACTLY);
        assertEquals(0, localizer.velocity().angVel, EXACTLY);
    }

    @Test
    public void bothSidesGoingAheadTakeItAheadTheWayItFaces() {
        tick();
        nugget.left.currentPosition = 5000;
        nugget.right.currentPosition = 5000;

        tick();

        assertEquals(10, localizer.pose().position.x, 1e-6);
        assertEquals(25, localizer.pose().position.y, 1e-6);
        assertEquals(Math.PI / 2, localizer.pose().heading.toDouble(), 1e-6);
    }

    @Test
    public void theRightSideGoingFurtherThanTheLeftTurnsItCounterclockwiseByTheDifferenceOverTheTrack() {
        tick();
        nugget.left.currentPosition = -1000;
        nugget.right.currentPosition = 1000;

        tick();

        assertEquals(10, localizer.pose().position.x, 1e-6);
        assertEquals(20, localizer.pose().position.y, 1e-6);
        assertEquals(Math.PI / 2 + 0.1, localizer.pose().heading.toDouble(), 1e-6);
    }

    @Test
    public void itsSpeedIsTheSidesSpeedAsTheRobotSeesIt() {
        tick();
        nugget.left.measuredVelocity = 1000;
        nugget.right.measuredVelocity = 3000;

        tick();

        assertEquals(2, localizer.velocity().linearVel.x, 1e-6);
        assertEquals(0, localizer.velocity().linearVel.y, EXACTLY);
        assertEquals(0.1, localizer.velocity().angVel, 1e-6);
    }

    @Test
    public void placingItSaysWhereItIsFromThenOn() {
        tick();
        localizer.setPose(new Pose2d(-30, 0, 0));
        nugget.left.currentPosition = 2000;
        nugget.right.currentPosition = 2000;

        tick();

        assertEquals(-28, localizer.pose().position.x, 1e-6);
        assertEquals(0, localizer.pose().position.y, 1e-6);
    }
}
