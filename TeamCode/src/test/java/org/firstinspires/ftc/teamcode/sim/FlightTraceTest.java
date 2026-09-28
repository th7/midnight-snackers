package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Flight;
import org.firstinspires.ftc.teamcode.simcore.Hives;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Vec2;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.junit.Test;

public class FlightTraceTest {
    private static final double POLLEN = 1.39;
    private static final double NECTAR = 1.8;
    private static final double STEP = 0.005;
    private static final int STEPS_PER_SAMPLE = 4;

    private final Field field = SimPlacement.FIELD;
    private Flight<Object> flight = Flight.over(Hives.of(field));
    private final List<Object> balls = new ArrayList<>();
    private final Map<Object, String> landed = new IdentityHashMap<>();
    private final FlightTrace trace = new FlightTrace();
    private double seconds = 0;

    private Object add(Field.Kind kind, double radius, double[] at, double[] velocity) {
        Object ball = new Object();
        balls.add(ball);
        flight = flight.with(ball, kind, Valid.value(Length.of(radius)), Points.vec(at), Points.vec(velocity));
        return ball;
    }

    private void fly(double duration) {
        for (long sample = Math.round(duration / (STEP * STEPS_PER_SAMPLE)); sample > 0; sample--) {
            for (int i = 0; i < STEPS_PER_SAMPLE; i++) {
                Flight.Stepped<Object> stepped = flight.step(Valid.value(Seconds.of(STEP)));
                flight = stepped.flight();
                for (Flight.Landing<Object> landing : stepped.landings()) {
                    Vec2 velocity =
                            landing instanceof Flight.OnTheFloor<Object> floor ? floor.velocity() : new Vec2(0, 0);
                    landed.put(
                            landing.ball(),
                            (landing instanceof Flight.OutOfTheField ? "out " : "landed ")
                                    + FlightTrace.format(landing.at().x()) + " "
                                    + FlightTrace.format(landing.at().y()) + " "
                                    + FlightTrace.format(landing.at().z()) + " "
                                    + FlightTrace.format(velocity.x()) + " "
                                    + FlightTrace.format(velocity.y()));
                }
                seconds += STEP;
            }
            sample();
        }
    }

    private void sample() {
        List<Object> values = new ArrayList<>();
        values.add(flight.hives().tilts().get("Blue"));
        values.add(flight.hives().tilts().get("Red"));
        values.add(flight.scored("Blue"));
        values.add(flight.scored("Red"));
        for (Object ball : balls) {
            values.add("|");
            if (flight.holds(ball)) {
                Vec3 at = flight.at(ball).orElseThrow();
                values.add(at.x());
                values.add(at.y());
                values.add(at.z());
            } else {
                values.add(landed.get(ball));
            }
        }
        trace.sample(seconds, values);
    }

    private double[] aboveTheMouthOf(String alliance, double height, double across) {
        Hives hives = flight.hives();
        Field.Hive hive = hives.hiveOf(alliance).orElseThrow();
        double[] mouth = Points.array(hives.upturnedCell(hive).orElseThrow().mouthCentreAt(hives.tilt(hive)));
        return new double[] {mouth[0], mouth[1] + across, mouth[2] + height};
    }

    @Test
    public void aHiveFilledWithNectarTipsAndWhatItHeldRollsOutAsItDidWhenTheTraceWasWritten() {
        for (int i = 0; i < 5; i++) {
            add(Field.Kind.NECTAR, NECTAR, aboveTheMouthOf("Blue", 10, (i - 2) * 1.5), new double[3]);
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
        add(Field.Kind.POLLEN, POLLEN, from, new double[] {
            (mouth[0] - from[0]) / seconds,
            (mouth[1] - from[1]) / seconds,
            rise / seconds + Flight.GRAVITY_IN_PER_S2 * seconds / 2
        });
        Field.Hive blue = flight.hives().hiveOf("Blue").orElseThrow();
        double[] onTheWall = Points.array(blue.at(flight.hives().tilt(blue), new Vec3(15.45, 10.03, 2.8)));
        add(Field.Kind.POLLEN, POLLEN, new double[] {onTheWall[0], onTheWall[1] + 6, onTheWall[2]}, new double[] {
            0, -200, 0
        });
        double half = field.size() / 2;
        add(Field.Kind.POLLEN, POLLEN, new double[] {half - 10, 30, 3}, new double[] {200, 0, 0});
        add(Field.Kind.POLLEN, POLLEN, new double[] {half - 10, -30, field.wallHeight() + 20}, new double[] {200, 0, 0
        });
        add(Field.Kind.NECTAR, NECTAR, new double[] {0, -60, 30}, new double[] {10, 5, 0});
        fly(3);

        trace.assertMatches("flightThrows");
    }
}
