package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Map;
import org.junit.Test;

public class LaunchTest {
    private static final double RPM = 1200.0 / 28 * 60;
    private static final double DELTA = 1e-6;
    private static final Length POLLEN = Valid.value(Length.of(1.4));
    private static final Launch.Robot STILL_AT_THE_ORIGIN =
            new Launch.Robot(new Vec2(0, 0), Valid.value(Heading.ofRadians(0)), new Vec2(0, 0), 0);

    private static Launch reginalds() {
        return Valid.value(RobotDescription.reginalds().launch());
    }

    private static Flywheels.Turning launching(double rpm) {
        return new Flywheels.Turning(Flywheels.Way.LAUNCHING, rpm, StrictMath.PI * 3.78 * rpm / 60);
    }

    private static Launch.Decision decide(
            Launch launch, Flywheels.Turning front, Flywheels.Turning back, Launch.Robot robot) {
        return Valid.value(launch.decide(front, back, POLLEN, robot));
    }

    private static WheelPair.Launched launched(Launch.Decision decision) {
        assertTrue("it launches: " + decision, decision instanceof Launch.Launches);
        return ((Launch.Launches) decision).launched();
    }

    @Test
    public void bothWheelsAtTheSameSpeedLaunchWithNoSpinAtTheMountingsAngle() {
        WheelPair.Launched launched =
                launched(decide(reginalds(), launching(RPM), launching(RPM), STILL_AT_THE_ORIGIN));

        assertEquals(WheelPair.SpinKind.NONE, launched.kind());
        assertEquals(174.06705108356715, launched.velocity().x(), DELTA);
        assertEquals(0, launched.velocity().y(), DELTA);
        assertEquals(478.2452923231548, launched.velocity().z(), DELTA);
    }

    @Test
    public void itLeavesFromTheExitPointTurnedWithTheRobotAndAlongItsHeading() {
        Launch.Robot facingRed = new Launch.Robot(
                new Vec2(10, 20), Valid.value(Heading.ofRadians(StrictMath.PI / 2)), new Vec2(0, 0), 0);

        WheelPair.Launched launched = launched(decide(reginalds(), launching(RPM), launching(RPM), facingRed));

        assertEquals(10, launched.at().x(), DELTA);
        assertEquals(16.74, launched.at().y(), DELTA);
        assertEquals(14, launched.at().z(), DELTA);
        assertEquals(0, launched.velocity().x(), DELTA);
        assertEquals(174.06705108356715, launched.velocity().y(), DELTA);
    }

    @Test
    public void aFasterFrontWheelGivesBackspinAndAFasterBackWheelTopspin() {
        WheelPair.Launched frontFaster =
                launched(decide(reginalds(), launching(RPM), launching(0.8 * RPM), STILL_AT_THE_ORIGIN));
        WheelPair.Launched backFaster =
                launched(decide(reginalds(), launching(0.8 * RPM), launching(RPM), STILL_AT_THE_ORIGIN));

        assertEquals(WheelPair.SpinKind.BACKSPIN, frontFaster.kind());
        assertEquals(WheelPair.SpinKind.TOPSPIN, backFaster.kind());
        assertEquals(347.142857142857, frontFaster.spinRpm(), DELTA);
        assertEquals(347.142857142857, backFaster.spinRpm(), DELTA);
    }

    @Test
    public void itCarriesTheVelocityOfTheExitPointOnTheRobot() {
        Launch.Robot drivingAndTurning =
                new Launch.Robot(new Vec2(0, 0), Valid.value(Heading.ofRadians(0)), new Vec2(10, 0), 1);

        WheelPair.Launched launched = launched(decide(reginalds(), launching(RPM), launching(RPM), drivingAndTurning));

        assertEquals(174.06705108356715 + 10, launched.velocity().x(), DELTA);
        assertEquals(-3.26, launched.velocity().y(), DELTA);
    }

    @Test
    public void aWheelStillOrTurningTheWrongWayKeepsThePollenAndSaysWhich() {
        Launch launch = reginalds();
        Flywheels.Turning wrongWay = new Flywheels.Turning(Flywheels.Way.WRONG_WAY, -RPM, -508.9);

        assertEquals(
                new Launch.Stays(Launch.Why.FRONT_WHEEL_WRONG_WAY),
                decide(launch, wrongWay, launching(RPM), STILL_AT_THE_ORIGIN));
        assertEquals(
                new Launch.Stays(Launch.Why.FRONT_WHEEL_STILL),
                decide(launch, Flywheels.Turning.still(), wrongWay, STILL_AT_THE_ORIGIN));
        assertEquals(
                new Launch.Stays(Launch.Why.BACK_WHEEL_WRONG_WAY),
                decide(launch, launching(RPM), wrongWay, STILL_AT_THE_ORIGIN));
        assertEquals(
                new Launch.Stays(Launch.Why.BACK_WHEEL_STILL),
                decide(launch, launching(RPM), Flywheels.Turning.still(), STILL_AT_THE_ORIGIN));
    }

    @Test
    public void aLaunchFasterThanAFlightTakesABallKeepsThePollen() {
        assertEquals(
                new Launch.Stays(Launch.Why.TOO_FAST),
                decide(reginalds(), launching(6000), launching(6000), STILL_AT_THE_ORIGIN));
    }

    @Test
    public void theMountingComesFromTheDescription() {
        Constants level = Valid.value(
                Constants.of(Map.of(Constant.LAUNCHER_PITCH_DEGREES, 0.0, Constant.LAUNCHER_HEIGHT_IN, 20.0)));
        Launch launch = Valid.value(RobotDescription.reginalds().with(level).launch());

        WheelPair.Launched launched = launched(decide(launch, launching(RPM), launching(RPM), STILL_AT_THE_ORIGIN));

        assertEquals(508.93800988154635, launched.velocity().x(), DELTA);
        assertEquals(0, launched.velocity().z(), DELTA);
        assertEquals(20, launched.at().z(), DELTA);
    }
}
