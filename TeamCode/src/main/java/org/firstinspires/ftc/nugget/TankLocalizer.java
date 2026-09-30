package org.firstinspires.ftc.nugget;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.roadrunner.ClockedOverflowEncoder;

public final class TankLocalizer implements Loopable {
    private final ClockedOverflowEncoder left;
    private final ClockedOverflowEncoder right;
    private final TankKinematics kinematics;
    private final double inPerTick;

    private Pose2d pose;
    private PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    private boolean counting = false;
    private int lastLeft;
    private int lastRight;

    public TankLocalizer(NuggetHardware hardware, Trajectories.Params params, Pose2d start) {
        this.left = encoderOf(hardware.left, DcMotorSimple.Direction.REVERSE, hardware.nanoClock);
        this.right = encoderOf(hardware.right, DcMotorSimple.Direction.FORWARD, hardware.nanoClock);
        this.kinematics = new TankKinematics(params.inPerTick * params.trackWidthTicks);
        this.inPerTick = params.inPerTick;
        this.pose = start;
    }

    private static ClockedOverflowEncoder encoderOf(
            DcMotorEx motor, DcMotorSimple.Direction mounted, LongSupplier nanoClock) {
        ClockedOverflowEncoder encoder = new ClockedOverflowEncoder(new RawEncoder(motor), nanoClock);
        encoder.setDirection(mounted);
        return encoder;
    }

    public Pose2d pose() {
        return pose;
    }

    public PoseVelocity2d velocity() {
        return velocity;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public void loop() {
        PositionVelocityPair leftRead = left.getPositionAndVelocity();
        PositionVelocityPair rightRead = right.getPositionAndVelocity();
        if (!counting) {
            counting = true;
            lastLeft = leftRead.position;
            lastRight = rightRead.position;
            return;
        }

        Twist2dDual<Time> twist = kinematics.forward(new TankKinematics.WheelIncrements<>(
                new DualNum<Time>(new double[] {leftRead.position - lastLeft, leftRead.velocity}).times(inPerTick),
                new DualNum<Time>(new double[] {rightRead.position - lastRight, rightRead.velocity}).times(inPerTick)));
        lastLeft = leftRead.position;
        lastRight = rightRead.position;

        pose = pose.plus(twist.value());
        velocity = twist.velocity().value();
    }
}
