package org.firstinspires.ftc.nugget;

import static org.junit.Assert.assertEquals;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.junit.Before;
import org.junit.Test;

public class NuggetTeleOpTest {
    private static final double EXACTLY = 1e-6;

    private final FakeDcMotorEx left = new FakeDcMotorEx();
    private final FakeDcMotorEx right = new FakeDcMotorEx();
    private final NuggetTeleOp teleOp = new NuggetTeleOp();

    @Before
    public void started() {
        teleOp.useHardware(NuggetHardware.builder().left(left).right(right).build());
        teleOp.telemetry = new FakeTelemetry();
        teleOp.gamepad1 = new Gamepad();
        teleOp.gamepad2 = new Gamepad();
        teleOp.init();
        teleOp.start();
    }

    @Test
    public void theLeftStickPushedForwardDrivesStraightAhead() {
        teleOp.gamepad1.left_stick_y = -0.6f;

        teleOp.loop();

        assertEquals(0.6, left.power, EXACTLY);
        assertEquals(0.6, right.power, EXACTLY);
    }

    @Test
    public void theRightStickPushedLeftTurnsCounterclockwise() {
        teleOp.gamepad1.right_stick_x = -0.4f;

        teleOp.loop();

        assertEquals(-0.4, left.power, EXACTLY);
        assertEquals(0.4, right.power, EXACTLY);
    }

    @Test
    public void sticksLetGoStopTheRobot() {
        teleOp.gamepad1.left_stick_y = -1;
        teleOp.loop();
        teleOp.gamepad1.left_stick_y = 0;

        teleOp.loop();

        assertEquals(0, left.power, EXACTLY);
        assertEquals(0, right.power, EXACTLY);
    }

    @Test
    public void theDriverStationListsItUnderNugget() {
        com.qualcomm.robotcore.eventloop.opmode.TeleOp listed =
                NuggetTeleOp.class.getAnnotation(com.qualcomm.robotcore.eventloop.opmode.TeleOp.class);

        assertEquals("Nugget TeleOp", listed.name());
        assertEquals("Nugget", listed.group());
    }
}
