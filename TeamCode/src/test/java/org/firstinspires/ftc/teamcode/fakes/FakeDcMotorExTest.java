package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.Assert.assertEquals;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import org.firstinspires.ftc.teamcode.simcore.Written;
import org.junit.Test;

public class FakeDcMotorExTest {
    private static final String FAKE = "org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx";

    private static StackTraceElement frame(String className, String method) {
        return new StackTraceElement(className, method, null, 0);
    }

    @Test
    public void theWriterIsTheFirstClassOutsideTheFake() {
        StackTraceElement[] frames = {
            frame(FAKE, "caller"), frame(FAKE, "setPower"), frame("org.example.Wheels", "set"),
        };

        assertEquals("org.example.Wheels", FakeDcMotorEx.callerOutside(frames, FAKE));
    }

    @Test
    public void anExtraHopInsideTheFakeDoesNotMakeTheFakeTheWriter() {
        StackTraceElement[] frames = {
            frame(FAKE, "caller"), frame(FAKE, "setPower"), frame(FAKE, "stop"), frame("org.example.Wheels", "set"),
        };

        assertEquals("org.example.Wheels", FakeDcMotorEx.callerOutside(frames, FAKE));
    }

    @Test
    public void aStackWithNobodyOutsideTheFakeNamesNobody() {
        StackTraceElement[] frames = {frame(FAKE, "caller"), frame(FAKE, "setPower")};

        assertEquals("unknown", FakeDcMotorEx.callerOutside(frames, FAKE));
    }

    @Test
    public void aMotorRecordsTheClassThatWroteIt() {
        FakeDcMotorEx motor = new FakeDcMotorEx();

        motor.setPower(1);

        assertEquals(List.of(FakeDcMotorExTest.class.getName()), motor.powerSetBy);
    }

    @Test
    public void writingThroughALambdaStillNamesTheEnclosingClass() {
        FakeDcMotorEx motor = new FakeDcMotorEx();

        ((Runnable) () -> motor.setPower(1)).run();

        assertEquals(List.of(FakeDcMotorExTest.class.getName()), motor.powerSetBy);
    }

    private static final String SDK_LYNX =
            "the SDK's LynxDcMotorController.setMotorVelocity (Hardware 11.2.1) switches the motor to"
                    + " RUN_USING_ENCODER unless it is in RUN_TO_POSITION";

    @Test
    public void writingAVelocityRunsTheMotorOnItsEncoderAsTheSdkDoes() {
        FakeDcMotorEx motor = new FakeDcMotorEx();

        motor.setVelocity(300);

        assertEquals(SDK_LYNX, DcMotor.RunMode.RUN_USING_ENCODER, motor.getMode());
    }

    @Test
    public void writingAVelocityLeavesRunToPositionAlone() {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setVelocity(300);

        assertEquals(SDK_LYNX, DcMotor.RunMode.RUN_TO_POSITION, motor.getMode());
    }

    @Test
    public void theHubIsGivenWhicheverWasWrittenLast() {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        assertEquals(new Written.Powered(0), motor.written());

        motor.setVelocity(300);
        assertEquals(new Written.AtVelocity(300), motor.written());

        motor.setPower(0.2);
        assertEquals(new Written.Powered(0.2), motor.written());
    }
}
