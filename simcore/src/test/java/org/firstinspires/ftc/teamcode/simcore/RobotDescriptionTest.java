package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import java.util.Map;
import org.junit.Test;

public class RobotDescriptionTest {
    @Test
    public void reginaldAsBuiltIsHisConfigurationHisHubsMountingAndEveryConstantAtItsDefault() {
        RobotDescription reginald = RobotDescription.reginalds();

        assertEquals(Configuration.reginalds(), reginald.configuration());
        assertEquals(HubMounting.reginalds(), reginald.hub());
        assertEquals(Constants.defaults(), reginald.constants());
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
}
