package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimField;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Before;
import org.junit.Test;

public class IntakeSimTest {
    private static final double LANE_Y = 0;

    private static final double START_X = -50;

    private static final double FIRST_PIECE_X = -30;

    private static final double PIECE_SPACING_IN = 10;

    private static final double PARKED_X = -60;

    private static final double PARKED_Y = -60;

    private static final float DRIVING_POWER = 0.5f;
    private static final double LOOP_SECONDS = 0.02;
    private static final int LOOPS = 250;

    private final SimRobot sim = new SimRobot();
    private final Robot robot = new Robot(sim.hardware(), Alliance.RELATIVE, new FakeTelemetry());

    @Before
    public void setUpTheLane() {
        sim.setPose(new Pose2d(START_X, LANE_Y, 0));
    }

    @Test
    public void theIntakeTakesInAPollenTheRobotDrivesInto() {
        emptyTheRobot();
        SimRobot.Piece pollen = placeInTheLane(loosePollen().get(0), FIRST_PIECE_X);

        driveDownTheLane();

        assertEquals("the pollen went into the hopper", 1, sim.held());
        assertNull("so it is nowhere on the field", sim.placeOf(pollen));
    }

    @Test
    public void aNectarCollidesAndStaysOnTheFloor() {
        emptyTheRobot();
        SimRobot.Piece nectar = placeInTheLane(aCellNectar(), FIRST_PIECE_X);

        driveDownTheLane();

        assertEquals("nothing went in", 0, sim.held());
        assertNotNull("the nectar is still on the floor", sim.placeOf(nectar));
        assertPushedAlong(nectar, FIRST_PIECE_X);
    }

    @Test
    public void theIntakeOffLeavesAPollenOnTheFloor() {
        emptyTheRobot();
        SimRobot.Piece pollen = placeInTheLane(loosePollen().get(0), FIRST_PIECE_X);
        robot.intake.off();

        driveDownTheLane();

        assertEquals("nothing went in", 0, sim.held());
        assertNotNull("the pollen is still on the floor", sim.placeOf(pollen));
        assertPushedAlong(pollen, FIRST_PIECE_X);
    }

    @Test
    public void aFullRobotTakesNothingIn() {
        assertEquals("the preload fills it", SimRobot.HOLDS, sim.held());
        SimRobot.Piece pollen = placeInTheLane(loosePollen().get(0), FIRST_PIECE_X);

        driveDownTheLane();

        assertEquals(SimRobot.HOLDS, sim.held());
        assertNotNull("the pollen is still on the floor", sim.placeOf(pollen));
        assertPushedAlong(pollen, FIRST_PIECE_X);
    }

    @Test
    public void theIntakeTakesInNoMoreThanTheRobotHolds() {
        emptyTheRobot();
        List<SimRobot.Piece> pollen = new ArrayList<>();
        for (int i = 0; i < SimRobot.HOLDS + 1; i++) {
            pollen.add(placeInTheLane(loosePollen().get(i), FIRST_PIECE_X + i * PIECE_SPACING_IN));
        }

        driveDownTheLane();

        assertEquals(SimRobot.HOLDS, sim.held());
        assertEquals("one pollen was left on the floor", 1, stillOnTheFloor(pollen));
    }

    private void driveDownTheLane() {
        for (int loop = 0; loop < LOOPS; loop++) {
            robot.intake.loop();
            robot.drive.manual(DRIVING_POWER, 0, 0);
            sim.step(LOOP_SECONDS);
        }
    }

    private SimRobot.Piece placeInTheLane(SimRobot.Piece piece, double x) {
        sim.place(piece, x, LANE_Y);
        return piece;
    }

    private void emptyTheRobot() {
        int parked = 0;
        for (SimRobot.Piece held : sim.holding()) {
            sim.place(held, PARKED_X + parked++ * PIECE_SPACING_IN, PARKED_Y);
        }
        assertEquals("the robot holds nothing now", 0, sim.held());
    }

    private List<SimRobot.Piece> loosePollen() {
        List<SimRobot.Piece> pollen = new ArrayList<>();
        for (SimRobot.Piece ball : sim.balls()) {
            if (SimField.POLLEN.equals(ball.kind())
                    && ball.setUpFrom()
                            .map(piece -> piece.cell == null && piece.flower == null)
                            .orElse(false)) {
                pollen.add(ball);
            }
        }
        assertTrue("the field has loose pollen to drive into", pollen.size() > SimRobot.HOLDS);
        return pollen;
    }

    private SimRobot.Piece aCellNectar() {
        for (SimRobot.Piece ball : sim.balls()) {
            if (SimField.NECTAR.equals(ball.kind())
                    && ball.setUpFrom().map(piece -> piece.cell != null).orElse(false)) {
                return ball;
            }
        }
        throw new AssertionError("the field has no nectar to put on the floor");
    }

    private int stillOnTheFloor(List<SimRobot.Piece> pieces) {
        int onTheFloor = 0;
        for (SimRobot.Piece piece : pieces) {
            if (sim.placeOf(piece) != null) {
                onTheFloor++;
            }
        }
        return onTheFloor;
    }

    private void assertPushedAlong(SimRobot.Piece piece, double placedAt) {
        double[] where = sim.placeOf(piece);
        assertTrue("the robot pushed it along rather than passing through it", where[0] > placedAt + 1);
        assertTrue("and it is still ahead of the robot", where[0] > sim.pose().position.x);
    }
}
