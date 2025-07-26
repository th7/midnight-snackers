package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.DriveRunner;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Drive extends SubSystem {
    private final Pose2d zeroPose = new Pose2d(0, 0, 0);
    private final MecanumDrive drive = new MecanumDrive(hardwareMap, zeroPose);
    private final DriveRunner driveRunner = new DriveRunner();

    public Drive(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
        telemetry.addData("Drive.init()", true);
    }

    public void loop() {
        driveRunner.loop();
        telemetry.addData("driveRunner.done()", driveRunner.done());
        telemetry.addData("drive.loop()", true);
    }

    @Override
    public boolean done() {
        return false;
    }

    public void drivePath(double[]... coordsList) {
        TrajectoryActionBuilder builder = drive.actionBuilder(getPose());
        for (double[] coords : coordsList) {
            builder = builder.splineToSplineHeading(new Pose2d(coords[0], coords[1], coords[2]), coords[2]);
        }
        Action action = builder.build();
        driveRunner.drive(action);
    }

    public void drivePath(Pose2d... poseList) {
        TrajectoryActionBuilder builder = drive.actionBuilder(getPose());
        for (Pose2d pose : poseList) {
            builder = builder.splineToSplineHeading(pose, pose.heading);
        }
        driveRunner.drive(builder.build());
    }

    public void splineUsingCoords() {
        drivePath(
                new double[]{24, 24, Math.PI / -2},
                new double[]{0, 48, Math.PI * -1}
        );
    }

    public void splineUsingPoses() {
        drivePath(
                new Pose2d(24, 24, Math.PI / -2),
                new Pose2d(0, 48, Math.PI * -1)
        );
    }

    public void splineBasic() {
        Action action = drive.actionBuilder(getPose())
                .splineToSplineHeading(new Pose2d(24, 24, Math.PI / -2), Math.PI / -2)
                .splineToSplineHeading(new Pose2d(0, 48, Math.PI * -1), Math.PI * -1)
                .build();
        driveRunner.drive(action);
    }

    private Pose2d getPose() {
        return drive.localizer.getPose();
    }

    public void setPose(Pose2d pose) {
        drive.localizer.setPose(pose);
    }
}
