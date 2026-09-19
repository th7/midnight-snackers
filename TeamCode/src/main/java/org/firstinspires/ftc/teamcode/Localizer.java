package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseEstimate;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

public class Localizer implements Loopable, PoseEstimate {
    public static final String CHANNEL = "Localizer";

    private final Prints telemetry;

    private static final int TRAIL = 100;

    private final TwoDeadWheelLocalizer deadWheels;
    private final LinkedList<Pose2d> trail = new LinkedList<>();
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);

    private PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

    public Localizer(
            DcMotorEx parallel,
            DcMotorEx perpendicular,
            LazyImu imu,
            Pose2d startingPose,
            LongSupplier clock,
            Prints telemetry) {
        this.telemetry = telemetry;
        deadWheels = new TwoDeadWheelLocalizer(
                parallel, perpendicular, imu.get(), MecanumDrive.PARAMS.inPerTick, startingPose, clock);
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
