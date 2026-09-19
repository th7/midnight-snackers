package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.Assert.assertEquals;

import java.util.List;
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
}
