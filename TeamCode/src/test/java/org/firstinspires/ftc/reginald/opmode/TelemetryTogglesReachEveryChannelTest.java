package org.firstinspires.ftc.reginald.opmode;

import static org.junit.Assert.assertEquals;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.LinkedHashSet;
import java.util.Set;
import org.firstinspires.ftc.reginald.Robot;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimDriverStation.State;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

public class TelemetryTogglesReachEveryChannelTest {
    private static class TestOp extends OpMode {
        TestOp() {
            super(Alliance.RELATIVE);
        }
    }

    private static TestOp initialised() {
        TestOp opMode = new TestOp();
        opMode.useHardware(new SimRobot().hardware());
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        return opMode;
    }

    private static void press(TestOp opMode, String button) {
        JsonObject pressed = new JsonObject();
        pressed.addProperty(button, true);
        State.fromJson(pressed).applyTo(opMode.gamepad2);
        opMode.loop();
        State.NEUTRAL.applyTo(opMode.gamepad2);
    }

    @Test
    public void everyChannelTheRobotMakesHasAButton() {
        TestOp opMode = initialised();
        Robot robot = opMode.robot;

        press(opMode, "cross");
        press(opMode, "square");
        press(opMode, "circle");

        Set<String> unreached = new LinkedHashSet<>(robot.channels.made());
        unreached.removeAll(robot.channels.areOn());
        assertEquals("channels the robot made that no button reaches", Set.of(), unreached);
    }
}
