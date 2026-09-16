package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.Arrays;
import java.util.List;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.hardware.Wheels;
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

@Config
public final class MecanumDrive {
    public static Params PARAMS = new Params();
    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);
    public final TurnConstraints defaultTurnConstraints =
            new TurnConstraints(PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint = new MinVelConstraint(Arrays.asList(
            kinematics.new WheelVelConstraint(PARAMS.maxWheelVel), new AngularVelConstraint(PARAMS.maxAngVel)));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);
    /** The four wheels, which this drive asks to turn and never reaches past. */
    public final Wheels wheels;

    public final VoltageSensor voltageSensor;
    public final LazyImu lazyImu;
    /** Where the robot is, how fast, and where it has been; this drive reads it and never updates it. */
    public final PoseEstimate where;

    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);
    /** The clock the trajectory followers run on: the robot's, so a simulation can own time. */
    private final LongSupplier clock;

    /** Seconds on the drive's clock, in place of {@code Actions.now()}. */
    private double now() {
        return clock.getAsLong() * 1e-9;
    }

    /**
     * Build the drive from already-resolved devices and whatever knows where the robot is, so it
     * can run on fakes as well as on the robot. The drive reads that and never updates it: whoever
     * owns the localizer decides when it moves on, once per loop.
     */
    public MecanumDrive(
            Wheels wheels, LazyImu lazyImu, VoltageSensor voltageSensor, PoseEstimate where, LongSupplier clock) {
        this.clock = clock;
        this.wheels = wheels;
        this.lazyImu = lazyImu;
        this.voltageSensor = voltageSensor;
        this.where = where;

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        wheels.drive(powers);
    }

    private void drawPoseHistory(Canvas c) {
        List<Pose2d> trail = where.trail();
        double[] xPoints = new double[trail.size()];
        double[] yPoints = new double[trail.size()];

        int i = 0;
        for (Pose2d t : trail) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(1e-6, new ProfileParams(0.25, 0.1, 1e-2)),
                beginPose,
                0.0,
                defaultTurnConstraints,
                defaultVelConstraint,
                defaultAccelConstraint);
    }

    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see
        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        // drive model parameters
        public double inPerTick = 0.0005352925;
        public double lateralInPerTick = 0.0004592815203259795;
        public double trackWidthTicks = 25311.699425025417;

        // feedforward parameters (in tick units)
        public double kS = 0.9891921306123841;
        public double kV = 0.0001281158359065848;
        public double kA = 0.00005;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 3.5;
        public double lateralGain = 5;
        public double headingGain = 5; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
        public double trajectoryTimeout = 10;
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private final double[] xPoints, yPoints;
        private double beginTs = -1;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(), Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = now();
                t = 0;
            } else {
                t = now() - beginTs;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
            PoseVelocity2d robotVelRobot = where.velocity();
            Pose2d error = txWorldTarget.value().minusExp(where.pose());

            if ((t >= timeTrajectory.duration && error.position.norm() < 0.1 && robotVelRobot.linearVel.norm() < 0.1)
                    || t >= timeTrajectory.duration + PARAMS.trajectoryTimeout) {

                wheels.stop();

                return false;
            }

            PoseVelocity2dDual<Time> command = new HolonomicController(
                            PARAMS.axialGain,
                            PARAMS.lateralGain,
                            PARAMS.headingGain,
                            PARAMS.axialVelGain,
                            PARAMS.lateralVelGain,
                            PARAMS.headingVelGain)
                    .compute(txWorldTarget, where.pose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward =
                    new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(
                    new MecanumCommandMessage(voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower));

            wheels.set(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

            p.put("x", where.pose().position.x);
            p.put("y", where.pose().position.y);
            p.put("heading (deg)", Math.toDegrees(where.pose().heading.toDouble()));

            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, where.pose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = now();
                t = 0;
            } else {
                t = now() - beginTs;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            Pose2d error = txWorldTarget.value().minusExp(where.pose());
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
            PoseVelocity2d robotVelRobot = where.velocity();

            if ((t >= turn.duration && error.heading.toDouble() < 0.1 && robotVelRobot.angVel < 0.1)
                    || t >= turn.duration + PARAMS.trajectoryTimeout) {
                wheels.stop();

                return false;
            }

            PoseVelocity2dDual<Time> command = new HolonomicController(
                            PARAMS.axialGain,
                            PARAMS.lateralGain,
                            PARAMS.headingGain,
                            PARAMS.axialVelGain,
                            PARAMS.lateralVelGain,
                            PARAMS.headingVelGain)
                    .compute(txWorldTarget, where.pose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward =
                    new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(
                    new MecanumCommandMessage(voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower));

            wheels.set(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, where.pose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }
}
