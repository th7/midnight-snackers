package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;

import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.junit.Test;

public class TheHubReportsAReversedMotorsCountNegatedTest {
    private static final String SDK_MOTOR =
            "https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/hardware/DcMotorImpl.html";
    private static final String ROAD_RUNNER_RAW_ENCODER = "https://github.com/acmerobotics/road-runner-ftc";
    private static final String WHY = "Road Runner's RawEncoder (" + ROAD_RUNNER_RAW_ENCODER
            + ") negates a reversed motor's count and velocity to undo the SDK's DcMotorImpl (" + SDK_MOTOR
            + "), whose getCurrentPosition and getVelocity negate them for a reversed motor";

    private static final int COUNTED = 1234;

    private static FakeDcMotorEx aMotorReporting(DcMotorSimple.Direction direction, int position, double velocity) {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        motor.setDirection(direction);
        motor.currentPosition = position;
        motor.measuredVelocity = velocity;
        return motor;
    }

    @Test
    public void roadRunnersRawEncoderNegatesAReversedMotorsCountAndVelocity() {
        FakeDcMotorEx reversed = aMotorReporting(DcMotorSimple.Direction.REVERSE, COUNTED, COUNTED);
        FakeDcMotorEx forward = aMotorReporting(DcMotorSimple.Direction.FORWARD, COUNTED, COUNTED);

        assertEquals(WHY, -COUNTED, new RawEncoder(reversed).getPositionAndVelocity().position);
        assertEquals(WHY, -COUNTED, (int) new RawEncoder(reversed).getPositionAndVelocity().velocity);
        assertEquals(WHY, COUNTED, new RawEncoder(forward).getPositionAndVelocity().position);
    }

    @Test
    public void soAFakeMotorHoldsWhatTheHubReportsAndTheEncoderReadsTheSameWhicheverWayItsMotorIsSet() {
        FakeDcMotorEx reversed = aMotorReporting(DcMotorSimple.Direction.REVERSE, -COUNTED, -COUNTED);
        FakeDcMotorEx forward = aMotorReporting(DcMotorSimple.Direction.FORWARD, COUNTED, COUNTED);

        assertEquals(WHY, COUNTED, new RawEncoder(reversed).getPositionAndVelocity().position);
        assertEquals(WHY, COUNTED, new RawEncoder(forward).getPositionAndVelocity().position);
    }
}
