package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

/**
 * Where the robot believes it is, and the sensors it believes it from: the two dead wheels and the
 * IMU. It moves that belief on <b>once</b> per loop and is the first subsystem ticked, so one tick
 * of the robot is one moment — everything that reads the pose during it reads the same pose, and
 * the encoders and the IMU are asked once between one loop and the next.
 *
 * <p>That it is once is the point. A second update inside the same loop asks the encoders again
 * for a delta that has barely happened, and leaves two subsystems that ran either side of it
 * disagreeing about where the robot was at a single instant. {@code LocalizerTicksOnceTest} holds
 * it to once.
 *
 * <p>What the belief <em>means</em> — the field, the alliance's half of it, where the goal is — is
 * {@link Nav}'s. This subsystem only keeps it current.
 */
public class Localizer extends SubSystem {
    private final TwoDeadWheelLocalizer deadWheels;
    private MecanumDrive mecanumDrive;

    /**
     * @param parallel the motor whose encoder port carries the parallel dead wheel
     * @param perpendicular the motor whose encoder port carries the perpendicular dead wheel
     * @param imu the hub's IMU, which the dead wheels take their heading from
     * @param startingPose where the robot believes it is until something says otherwise
     * @param clock the robot's clock, which the dead wheels measure their own speed against
     */
    public Localizer(
            DcMotorEx parallel, DcMotorEx perpendicular, LazyImu imu, Pose2d startingPose, LongSupplier clock) {
        deadWheels = new TwoDeadWheelLocalizer(
                parallel, perpendicular, imu.get(), MecanumDrive.PARAMS.inPerTick, startingPose, clock);
    }

    /**
     * The dead wheels themselves, for the drive Road Runner follows trajectories with and for its
     * tuning op modes, which read the encoders directly.
     */
    public TwoDeadWheelLocalizer deadWheels() {
        return deadWheels;
    }

    @Override
    protected void onInit() {
        mecanumDrive = robot.mecanumDrive;
    }

    /**
     * The one pose update of the loop. It goes through the drive rather than the dead wheels
     * directly so that the dashboard's trail and the flight recorder's estimate are written with
     * it, as they were when a trajectory being followed did this itself.
     */
    @Override
    protected void onLoop() {
        mecanumDrive.updatePoseEstimate();
    }

    @Override
    protected void onTelemetry() {
        Pose2d pose = pose();
        telemetry.addData("localizer.x", pose.position.x);
        telemetry.addData("localizer.y", pose.position.y);
        telemetry.addData("localizer.heading (deg)", Math.toDegrees(pose.heading.toDouble()));
    }

    /** Where the robot believes it is, as of this loop's update. */
    public Pose2d pose() {
        return deadWheels.getPose();
    }

    /** Tells the robot where it is, e.g. where it was placed before an auto, or what the camera saw. */
    public void setPose(Pose2d pose) {
        deadWheels.setPose(pose);
    }
}
