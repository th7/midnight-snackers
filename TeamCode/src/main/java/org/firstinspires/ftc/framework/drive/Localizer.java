package org.firstinspires.ftc.framework.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import org.firstinspires.ftc.framework.drive.messages.PoseMessage;
import org.firstinspires.ftc.framework.hardware.Hardware;
import org.firstinspires.ftc.framework.robot.Prints;
import org.firstinspires.ftc.framework.robot.SubSystem;

public class Localizer extends SubSystem implements PoseEstimate {
    private final Prints telemetry;

    private static final int TRAIL = 100;

    private final TwoDeadWheelLocalizer deadWheels;
    private final LinkedList<Pose2d> trail = new LinkedList<>();
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);

    private PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

    public Localizer(Hardware hardware, Prints telemetry) {
        this.telemetry = telemetry;
        deadWheels = new TwoDeadWheelLocalizer(
                hardware.motor("rightBack"),
                hardware.motor("leftFront"),
                hardware.imu().get(),
                MecanumDrive.PARAMS.inPerTick,
                new Pose2d(0, 0, 0),
                hardware.nanoClock());
    }

    public TwoDeadWheelLocalizer deadWheels() {
        return deadWheels;
    }

    public void update() {
        velocity = deadWheels.update();
        trail.add(deadWheels.getPose());
        while (trail.size() > TRAIL) {
            trail.removeFirst();
        }
        estimatedPoseWriter.write(new PoseMessage(deadWheels.getPose()));
    }

    @Override
    public Pose2d pose() {
        return deadWheels.getPose();
    }

    @Override
    public PoseVelocity2d velocity() {
        return velocity;
    }

    @Override
    public List<Pose2d> trail() {
        return Collections.unmodifiableList(trail);
    }

    public void setPose(Pose2d pose) {
        deadWheels.setPose(pose);
    }

    @Override
    public void loop() {
        update();

        Pose2d pose = pose();
        telemetry.addData("localizer.x", pose.position.x);
        telemetry.addData("localizer.y", pose.position.y);
        telemetry.addData("localizer.heading (deg)", Math.toDegrees(pose.heading.toDouble()));
    }
}
