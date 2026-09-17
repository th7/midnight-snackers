package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.Assert.assertEquals;

import java.util.List;
import org.junit.Test;

/**
 * A fake motor remembers who wrote it, which is how {@code DriveOwnsTheWheelsTest} asks the four
 * wheels whether anyone but {@code Wheels} has had their hands on them. Getting that attribution
 * right is the whole of that rule, so it is held here rather than assumed there.
 */
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

    /**
     * The case counting frames got wrong: one more hop inside the fake and {@code frames[2]} is
     * the fake itself, so every motor looks as though the fake wrote it and the rule passes while
     * saying nothing.
     */
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

    /** And end to end: the class that actually called is the one recorded. */
    @Test
    public void aMotorRecordsTheClassThatWroteIt() {
        FakeDcMotorEx motor = new FakeDcMotorEx();

        motor.setPower(1);

        assertEquals(List.of(FakeDcMotorExTest.class.getName()), motor.powerSetBy);
    }

    /** Including through a lambda, whose frame belongs to the class that wrote it. */
    @Test
    public void writingThroughALambdaStillNamesTheEnclosingClass() {
        FakeDcMotorEx motor = new FakeDcMotorEx();

        ((Runnable) () -> motor.setPower(1)).run();

        assertEquals(List.of(FakeDcMotorExTest.class.getName()), motor.powerSetBy);
    }
}
