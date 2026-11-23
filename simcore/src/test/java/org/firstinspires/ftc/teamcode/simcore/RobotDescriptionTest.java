package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Map;
import org.junit.Test;

public class RobotDescriptionTest {
    @Test
    public void reginaldAsBuiltIsHisConfigurationHisHubsMountingAndEveryConstantAtItsDefault() {
        RobotDescription reginald = RobotDescription.reginalds();

        assertEquals(Configuration.reginalds(), reginald.configuration());
        assertEquals(HubMounting.reginalds(), reginald.hub());
        assertEquals(Constants.defaults(), reginald.constants());
        assertEquals(Directions.reginalds(), reginald.directions());
    }

    @Test
    public void constantsSetForARunReplaceOnlyTheConstants() {
        Constants heavier = Valid.value(Constants.of(Map.of(Constant.MASS_KG, 18.0)));

        RobotDescription run = RobotDescription.reginalds().with(heavier);

        assertEquals(heavier, run.constants());
        assertEquals(HubMounting.reginalds(), run.hub());
        assertEquals(Configuration.reginalds(), run.configuration());
    }

    @Test
    public void reginaldsDriveAsBuiltRunsFreeWhereItsTunedFeedforwardSays() {
        Drivetrain drive = Valid.value(RobotDescription.reginalds().drivetrain(PerWheel.all(Noise.Motor.tuned())));
        double freeInchesPerSecond = 46.69371123756329;

        PerWheel<Double> accelerations = Valid.value(drive.accelerations(
                PerWheel.all(new Drivetrain.Setting(Power.clamped(1), ZeroPower.BRAKE))
                        .with(Wheel.LEFT_BACK, new Drivetrain.Setting(Power.clamped(-1), ZeroPower.BRAKE))
                        .with(Wheel.RIGHT_BACK, new Drivetrain.Setting(Power.clamped(-1), ZeroPower.BRAKE)),
                12,
                PerWheel.all(freeInchesPerSecond),
                Valid.value(Seconds.of(0.005)),
                Traction.unlimited()));

        for (Wheel wheel : Wheel.values()) {
            assertEquals(wheel.name(), 0, accelerations.of(wheel), 1e-9);
        }
    }

    @Test
    public void reginaldsDeadWheelsAsBuiltSitWhereTheDescriptionPutsThem() {
        DeadWheels wheels = Valid.value(RobotDescription.reginalds().deadWheels());

        DeadWheels.Counts counts = wheels.moved(new Twist(new Vec2(0, 0), 0.1)).counts(new Twist(new Vec2(0, 0), 0));

        assertEquals(46.0148473347004, counts.par().ticks(), 1e-9);
        assertEquals(-644.2078626858056, counts.perp().ticks(), 1e-9);
    }

    @Test
    public void whatDiffersFromReginaldAsBuiltIsSaidPartByPart() {
        RobotDescription built = RobotDescription.reginalds();
        RobotDescription turnedHub = RobotDescription.of(
                Configuration.reginalds(),
                Valid.value(HubMounting.of(Facing.DOWN, Facing.RIGHT)),
                Constants.defaults(),
                Directions.reginalds());
        RobotDescription noIntake = RobotDescription.of(
                Valid.value(Configuration.of(java.util.List.of(new Configuration.Imu("imu")))),
                HubMounting.reginalds(),
                Constants.defaults(),
                Directions.reginalds());
        RobotDescription heavier = built.with(Valid.value(Constants.of(Map.of(Constant.MASS_KG, 18.0))));
        RobotDescription intakeReversed = built.with(Directions.reginalds().with(Mechanism.INTAKE, true));

        assertTrue(built.asBuilt());
        assertFalse(turnedHub.asBuilt());
        assertFalse(turnedHub.hubAsBuilt());
        assertTrue(turnedHub.configurationAsBuilt());
        assertFalse(noIntake.configurationAsBuilt());
        assertTrue(noIntake.hubAsBuilt());
        assertFalse(heavier.asBuilt());
        assertTrue(heavier.hubAsBuilt() && heavier.configurationAsBuilt() && heavier.directionsAsBuilt());
        assertFalse(intakeReversed.asBuilt());
        assertFalse(intakeReversed.directionsAsBuilt());
        assertTrue(intakeReversed.constants().asBuilt() && intakeReversed.hubAsBuilt());
    }

    @Test
    public void theSpindexerIsAsBuiltWhileItsSpinnerTurnsAsBuiltAndNoneOfItsConstantsHasChanged() {
        RobotDescription built = RobotDescription.reginalds();
        RobotDescription spinnerReversed = built.with(Directions.reginalds().with(Mechanism.SPINNER, true));
        RobotDescription quickerRamp =
                built.with(Valid.value(Constants.of(Map.of(Constant.RAMP_SECONDS_PER_60_DEGREES, 0.1))));
        RobotDescription intakeReversed = built.with(Directions.reginalds().with(Mechanism.INTAKE, true));
        RobotDescription heavier = built.with(Valid.value(Constants.of(Map.of(Constant.MASS_KG, 18.0))));

        assertTrue(built.spindexerAsBuilt());
        assertFalse(spinnerReversed.spindexerAsBuilt());
        assertFalse(quickerRamp.spindexerAsBuilt());
        assertTrue(intakeReversed.spindexerAsBuilt());
        assertTrue(heavier.spindexerAsBuilt());
    }

    @Test
    public void theLauncherIsAsBuiltWhileItsFlywheelsTurnAsBuiltAndNoneOfItsConstantsHasChanged() {
        RobotDescription built = RobotDescription.reginalds();
        RobotDescription frontReversed = built.with(Directions.reginalds().with(Mechanism.FLYWHEEL_FRONT, true));
        RobotDescription backReversed = built.with(Directions.reginalds().with(Mechanism.FLYWHEEL_BACK, true));
        RobotDescription heavierWheel = built.with(
                Valid.value(Constants.of(Map.of(Constant.FLYWHEEL_FRONT_KA_VOLT_SECONDS_SQUARED_PER_TICK, 0.01))));
        RobotDescription spinnerReversed = built.with(Directions.reginalds().with(Mechanism.SPINNER, true));
        RobotDescription quickerRamp =
                built.with(Valid.value(Constants.of(Map.of(Constant.RAMP_SECONDS_PER_60_DEGREES, 0.1))));

        assertTrue(built.launcherAsBuilt());
        assertFalse(frontReversed.launcherAsBuilt());
        assertFalse(backReversed.launcherAsBuilt());
        assertFalse(heavierWheel.launcherAsBuilt());
        assertTrue(spinnerReversed.launcherAsBuilt());
        assertTrue(quickerRamp.launcherAsBuilt());
        assertTrue(heavierWheel.spindexerAsBuilt());
    }
}
