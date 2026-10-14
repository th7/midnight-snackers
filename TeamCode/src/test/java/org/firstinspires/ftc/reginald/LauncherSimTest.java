package org.firstinspires.ftc.reginald;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimNoise;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.Constants;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Launch;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.junit.Test;

public class LauncherSimTest {
    private final SimRobot sim = new SimRobot();
    private final Robot robot = new Robot(sim.hardware(), Alliance.BLUE, new FakeTelemetry());

    @Test
    public void aCloseLaunchFromTheLaunchDistanceScoresOneBallInTheBlueHive() {
        Field.Cell cell = sim.upturnedCell("Blue");
        int already = sim.scored("Blue");
        sim.setPose(facing(cell, Nav.LAUNCH_DISTANCE_INCHES));
        robot.launcher.setCloseLaunchPower();

        robot.launcher.launchyLaunch();
        int loops = 0;
        while (!robot.launcher.launchDone() && loops++ < 200) {
            robot.launcher.loop();
            sim.step(0.02);
        }
        assertTrue("the launch sequence finished", robot.launcher.launchDone());
        sim.step(2.0);

        assertEquals(already + 1, sim.scored("Blue"));
        assertEquals(SimRobot.PRELOAD - 1, sim.held());
    }

    @Test
    public void thePreloadEmptiesTheRobotAndFillsTheHive() {
        Field.Cell cell = sim.upturnedCell("Blue");
        double leaning = sim.tilt("Blue");
        assertEquals("three fifths full to start with", 0.6, sim.load("Blue"), 0.001);
        sim.setPose(facing(cell, Nav.LAUNCH_DISTANCE_INCHES));
        robot.launcher.setCloseLaunchPower();

        for (int launch = 0; launch < SimRobot.PRELOAD; launch++) {
            robot.launcher.launchyLaunch();
            int loops = 0;
            while (!robot.launcher.launchDone() && loops++ < 200) {
                robot.launcher.loop();
                sim.step(0.02);
            }
        }
        assertEquals("all four went", 0, sim.held());

        sim.step(3.0);

        assertEquals("the hive tipped", -leaning, sim.tilt("Blue"), 0.001);
        assertEquals("and the cell that went under dropped what was in it", 0, sim.scored("Blue"));
    }

    @Test
    public void fourLaunchesAtTheirCadenceAllComeToRestInAnEmptyCellAtTheBuiltThrowAndTwoInchesASecondEitherSide() {
        double twoInchesASecond = 2.1 / CODES_CLOSE_LAUNCH_TICKS_PER_SECOND;
        for (double throwing : new double[] {
            Launch.IN_PER_S_PER_TICK_PER_S,
            Launch.IN_PER_S_PER_TICK_PER_S - twoInchesASecond,
            Launch.IN_PER_S_PER_TICK_PER_S + twoInchesASecond
        }) {
            String said = "throwing " + throwing * CODES_CLOSE_LAUNCH_TICKS_PER_SECOND + " in/s";
            SimRobot thrower = new SimRobot(TeamRobot.REGINALD, SimNoise.NONE, constants(throwing));
            Robot code = new Robot(thrower.hardware(), Alliance.BLUE, new FakeTelemetry());
            int aside = 0;
            for (SimRobot.Piece nectar : thrower.balls()) {
                if (nectar.setUpFrom()
                                .filter(piece -> piece.place() instanceof Field.Place.InCell)
                                .isPresent()
                        && nectar.setUpFrom().get().alliance().equals(Optional.of("Blue"))) {
                    thrower.place(nectar, -60, -60 + 4 * aside++);
                }
            }
            Field.Cell cell = thrower.upturnedCell("Blue");
            thrower.setPose(facing(thrower, cell, Nav.LAUNCH_DISTANCE_INCHES));
            code.launcher.setCloseLaunchPower();

            for (int launch = 0; launch < SimRobot.PRELOAD; launch++) {
                code.launcher.launchyLaunch();
                int loops = 0;
                while (!code.launcher.launchDone() && loops++ < 200) {
                    code.launcher.loop();
                    thrower.step(0.02);
                }
            }
            thrower.step(3.0);
            List<double[]> resting = new ArrayList<>();
            for (SimRobot.Piece preloaded : thrower.balls()) {
                if (preloaded.setUpFrom().isEmpty()) {
                    resting.add(thrower.placeOf(preloaded));
                }
            }
            thrower.step(1.0);

            assertEquals(said + ": all four went in", SimRobot.PRELOAD, thrower.scored("Blue"));
            int i = 0;
            for (SimRobot.Piece preloaded : thrower.balls()) {
                if (preloaded.setUpFrom().isEmpty()) {
                    assertArrayEquals(said + ": and stay at rest", resting.get(i++), thrower.placeOf(preloaded), 1e-3);
                }
            }
        }
    }

    private static final double CODES_CLOSE_LAUNCH_TICKS_PER_SECOND = 1050;

    private static Constants constants(double throwing) {
        Map<Constant, Double> set = new EnumMap<>(Constant.class);
        set.put(Constant.LAUNCH_THROW, throwing);
        return Constants.of(set).fold(constants -> constants, rule -> {
            throw new AssertionError(rule);
        });
    }

    private Pose2d facing(Field.Cell cell, double distance) {
        return facing(sim, cell, distance);
    }

    private static Pose2d facing(SimRobot sim, Field.Cell cell, double distance) {
        Vec3 centre = cell.mouthCentreAt(sim.tilt(cell.alliance()));
        Vec3 normal = cell.mouthNormalAt(sim.tilt(cell.alliance()));
        double length = Math.hypot(normal.x(), normal.y());
        double nx = normal.x() / length, ny = normal.y() / length;
        return new Pose2d(centre.x() + nx * distance, centre.y() + ny * distance, Math.atan2(-ny, -nx));
    }
}
