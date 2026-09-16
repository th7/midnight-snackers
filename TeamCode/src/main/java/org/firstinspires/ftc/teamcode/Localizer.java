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
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseEstimate;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

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
 * <p>It keeps what that update measures — the pose, the speed, and the recent trail — because all
 * three are answers about where the robot is, and something that only follows a path should be
 * asking rather than working them out again.
 *
 * <p>What the belief <em>means</em> — the field, the alliance's half of it, where the goal is — is
 * {@link Nav}'s. This subsystem only keeps it current.
 */
public class Localizer extends SubSystem implements PoseEstimate {
    /** How much of the trail is kept: enough to see where the robot has just been. */
    private static final int TRAIL = 100;

    private final TwoDeadWheelLocalizer deadWheels;
    private final LinkedList<Pose2d> trail = new LinkedList<>();
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);

    private PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

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
     * The dead wheels themselves, for Road Runner's tuning op modes, which read the encoders
     * directly.
     */
    public TwoDeadWheelLocalizer deadWheels() {
        return deadWheels;
    }

    @Override
    protected void onInit() {}

    @Override
    protected void onLoop() {
        update();
    }

    /**
     * Reads the dead wheels and the IMU once and moves the pose, the speed and the trail on. The
     * loop calls this; so do Road Runner's tuning op modes, which run without a robot around them.
     */
    public void update() {
        velocity = deadWheels.update();
        trail.add(deadWheels.getPose());
        while (trail.size() > TRAIL) {
            trail.removeFirst();
        }
        estimatedPoseWriter.write(new PoseMessage(deadWheels.getPose()));
    }

    @Override
    protected void onTelemetry() {
        Pose2d pose = pose();
        telemetry.addData("localizer.x", pose.position.x);
        telemetry.addData("localizer.y", pose.position.y);
        telemetry.addData("localizer.heading (deg)", Math.toDegrees(pose.heading.toDouble()));
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

    /** Tells the robot where it is, e.g. where it was placed before an auto, or what the camera saw. */
    public void setPose(Pose2d pose) {
        deadWheels.setPose(pose);
    }
}
