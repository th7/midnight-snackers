package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = TuningDrive.from(hardwareMap, new Pose2d(0, 0, 0)).drive;

        if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
            throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }
        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .lineToX(DISTANCE)
                    .lineToX(0)
                    .build());
        }
    }
}
