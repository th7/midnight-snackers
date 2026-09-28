package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import org.junit.Test;

public class FlightTraceTest {
    private static final double POLLEN = 1.39;
    private static final double NECTAR = 1.8;
    private static final double STEP = 0.005;
    private static final int STEPS_PER_SAMPLE = 4;

    private final SimField field = SimPlacement.FIELD;
    private final SimHives hives = new SimHives(field);
    private final SimFlight flight = new SimFlight(field, hives);
    private final List<Object> balls = new ArrayList<>();
    private final Map<Object, String> landed = new IdentityHashMap<>();
    private final FlightTrace trace = new FlightTrace();
    private double seconds = 0;

    private Object add(String kind, double radius, double[] at, double[] velocity) {
        Object ball = new Object();
        balls.add(ball);
        flight.add(ball, kind, radius, at, velocity);
        return ball;
    }

    private void fly(double duration) {
        for (long sample = Math.round(duration / (STEP * STEPS_PER_SAMPLE)); sample > 0; sample--) {
            for (int i = 0; i < STEPS_PER_SAMPLE; i++) {
                for (SimFlight.Landing landing : flight.step(STEP)) {
                    landed.put(
                            landing.ball,
                            (landing.out ? "out " : "landed ")
                                    + FlightTrace.format(landing.at[0]) + " " + FlightTrace.format(landing.at[1])
                                    + " " + FlightTrace.format(landing.at[2]) + " "
                                    + FlightTrace.format(landing.velocity[0]) + " "
                                    + FlightTrace.format(landing.velocity[1]));
                }
                seconds += STEP;
            }
            sample();
        }
    }

    private void sample() {
        List<Object> values = new ArrayList<>();
        values.add(hives.tilt("Blue"));
        values.add(hives.tilt("Red"));
        values.add(flight.scored("Blue"));
        values.add(flight.scored("Red"));
        for (Object ball : balls) {
            values.add("|");
            if (flight.holds(ball)) {
                double[] at = flight.at(ball);
                values.add(at[0]);
                values.add(at[1]);
                values.add(at[2]);
            } else {
                values.add(landed.get(ball));
            }
        }
        trace.sample(seconds, values);
    }

    private double[] aboveTheMouthOf(String alliance, double height, double across) {
        double[] mouth = hives.upturnedCell(alliance).mouthCentreAt(hives.tilt(alliance));
        return new double[] {mouth[0], mouth[1] + across, mouth[2] + height};
    }

    @Test
    public void aHiveFilledWithNectarTipsAndWhatItHeldRollsOutAsItDidWhenTheTraceWasWritten() {
        for (int i = 0; i < 5; i++) {
            add(SimField.NECTAR, NECTAR, aboveTheMouthOf("Blue", 10, (i - 2) * 1.5), new double[3]);
            fly(0.4);
        }
        fly(5);

        trace.assertMatches("flightFillsAndTipsAHive");
    }

    @Test
    public void ballsThrownAtTheHivesTheWallsAndTheFloorFlyAsTheyDidWhenTheTraceWasWritten() {
        double[] mouth = aboveTheMouthOf("Red", 0, 0);
        double[] from = {mouth[0] - 40, mouth[1] - 10, SimRobot.LAUNCH_HEIGHT_IN};
        double rise = mouth[2] + 4 - from[2], seconds = 0.6;
        add(SimField.POLLEN, POLLEN, from, new double[] {
            (mouth[0] - from[0]) / seconds,
            (mouth[1] - from[1]) / seconds,
            rise / seconds + SimFlight.GRAVITY_IN_PER_S2 * seconds / 2
        });
        double[] onTheWall = hives.hiveOf("Blue").at(hives.tilt("Blue"), new double[] {15.45, 10.03, 2.8});
        add(SimField.POLLEN, POLLEN, new double[] {onTheWall[0], onTheWall[1] + 6, onTheWall[2]}, new double[] {
            0, -200, 0
        });
        double half = field.size / 2;
        add(SimField.POLLEN, POLLEN, new double[] {half - 10, 30, 3}, new double[] {200, 0, 0});
        add(SimField.POLLEN, POLLEN, new double[] {half - 10, -30, field.wallHeight + 20}, new double[] {200, 0, 0});
        add(SimField.NECTAR, NECTAR, new double[] {0, -60, 30}, new double[] {10, 5, 0});
        fly(3);

        trace.assertMatches("flightThrows");
    }
}
