package org.firstinspires.ftc.nugget;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.Arrays;
import java.util.List;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;

@Config
public final class Trajectories {
    public static Params PARAMS = new Params();

    public static class Params {
        public double inPerTick = 0.0005352925;
        public double trackWidthTicks = 50623.398850050834;

        public double kSVolts = 0.9891921306123841;
        public double kVVoltSecondsPerTick = 0.0001281158359065848;
        public double kAVoltSecondsSquaredPerTick = 0.00005;

        public double maxWheelVelInchesPerSecond = 30;
        public double minProfileAccelInchesPerSecondSquared = -30;
        public double maxProfileAccelInchesPerSecondSquared = 30;

        public double maxAngVelRadiansPerSecond = Math.PI / 2;
        public double maxAngAccelRadiansPerSecondSquared = Math.PI / 2;

        public double ramseteZeta = 0.7;
        public double ramseteBBar = 2.0;

        public double turnGain = 3;
        public double turnVelGain = 0;
    }

    private final TankDrive tank;
    private final TankLocalizer localizer;
    private final VoltageSensor battery;
    private final LongSupplier nanoClock;
    private final Params params;

    private final TankKinematics kinematics;
    private final TurnConstraints turnConstraints;
    private final VelConstraint velConstraint;
    private final AccelConstraint accelConstraint;

    public Trajectories(TankDrive tank, TankLocalizer localizer, NuggetHardware hardware, Params params) {
        this.tank = tank;
        this.localizer = localizer;
        this.battery = hardware.voltageSensor;
        this.nanoClock = hardware.nanoClock;
        this.params = params;

        this.kinematics = new TankKinematics(params.inPerTick * params.trackWidthTicks);
        this.turnConstraints = new TurnConstraints(
                params.maxAngVelRadiansPerSecond,
                -params.maxAngAccelRadiansPerSecondSquared,
                params.maxAngAccelRadiansPerSecondSquared);
        this.velConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(params.maxWheelVelInchesPerSecond),
                new AngularVelConstraint(params.maxAngVelRadiansPerSecond)));
        this.accelConstraint = new ProfileAccelConstraint(
                params.minProfileAccelInchesPerSecondSquared, params.maxProfileAccelInchesPerSecondSquared);
    }

    public TrajectoryActionBuilder from(Pose2d begin) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(1e-6, new ProfileParams(0.25, 0.1, 1e-2)),
                begin,
                0.0,
                turnConstraints,
                velConstraint,
                accelConstraint);
    }

    private double now() {
        return nanoClock.getAsLong() * 1e-9;
    }

    private void drive(PoseVelocity2dDual<Time> command) {
        TankKinematics.WheelVelocities<Time> sides = kinematics.inverse(command);
        MotorFeedforward feedforward = new MotorFeedforward(
                params.kSVolts,
                params.kVVoltSecondsPerTick / params.inPerTick,
                params.kAVoltSecondsSquaredPerTick / params.inPerTick);
        double volts = battery.getVoltage();
        tank.sides(feedforward.compute(sides.left) / volts, feedforward.compute(sides.right) / volts);
    }

    private void draw(TelemetryPacket packet, Pose2d target) {
        Pose2d pose = localizer.pose();
        packet.put("x", pose.position.x);
        packet.put("y", pose.position.y);
        packet.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

        Canvas canvas = packet.fieldOverlay();
        canvas.setStroke("#4CAF50");
        Drawing.drawRobot(canvas, target);
        canvas.setStroke("#3F51B5");
        Drawing.drawRobot(canvas, pose);
    }

    public final class FollowTrajectoryAction implements Action {
        private final TimeTrajectory trajectory;
        private final double[] xPoints;
        private final double[] yPoints;
        private double beganAt = -1;

        public FollowTrajectoryAction(TimeTrajectory trajectory) {
            this.trajectory = trajectory;

            List<Double> along = com.acmerobotics.roadrunner.Math.range(
                    0, trajectory.path.length(), Math.max(2, (int) Math.ceil(trajectory.path.length() / 2)));
            xPoints = new double[along.size()];
            yPoints = new double[along.size()];
            for (int i = 0; i < along.size(); i++) {
                Pose2d point = trajectory.path.get(along.get(i), 1).value();
                xPoints[i] = point.position.x;
                yPoints[i] = point.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (beganAt < 0) {
                beganAt = now();
            }
            double t = now() - beganAt;
            if (t >= trajectory.duration) {
                tank.stop();
                return false;
            }

            DualNum<Time> travelled = trajectory.profile.get(t);
            Pose2dDual<Arclength> target = trajectory.path.get(travelled.value(), 3);
            PoseVelocity2d steered = new RamseteController(
                            kinematics.trackWidth, params.ramseteZeta, params.ramseteBBar)
                    .compute(travelled, target, localizer.pose())
                    .value();
            DualNum<Time> turning = target.reparam(travelled).heading.velocity();
            drive(new PoseVelocity2dDual<>(
                    new Vector2dDual<>(
                            new DualNum<Time>(new double[] {steered.linearVel.x, travelled.get(2)}),
                            DualNum.constant(0.0, 2)),
                    new DualNum<Time>(new double[] {steered.angVel, turning.get(1)})));

            draw(packet, target.value());
            Canvas canvas = packet.fieldOverlay();
            canvas.setStroke("#4CAF50FF");
            canvas.setStrokeWidth(1);
            canvas.strokePolyline(xPoints, yPoints);
            return true;
        }

        @Override
        public void preview(@NonNull Canvas canvas) {
            canvas.setStroke("#4CAF507A");
            canvas.setStrokeWidth(1);
            canvas.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;
        private double beganAt = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (beganAt < 0) {
                beganAt = now();
            }
            double t = now() - beganAt;
            if (t >= turn.duration) {
                tank.stop();
                return false;
            }

            Pose2dDual<Time> target = turn.get(t);
            double headingError = target.heading.value().minus(localizer.pose().heading);
            double turningError = target.heading.velocity().value() - localizer.velocity().angVel;
            drive(new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3),
                    target.heading
                            .velocity()
                            .plus(params.turnGain * headingError + params.turnVelGain * turningError)));

            draw(packet, target.value());
            Canvas canvas = packet.fieldOverlay();
            canvas.setStroke("#7C4DFFFF");
            canvas.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
            return true;
        }

        @Override
        public void preview(@NonNull Canvas canvas) {
            canvas.setStroke("#7C4DFF7A");
            canvas.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }
}
