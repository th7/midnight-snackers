package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.Drawing;

import java.util.function.Supplier;

public class DriveRunner {
    public final FtcDashboard dash = FtcDashboard.getInstance();
    private Action roadRunnerAction = null;
    private Supplier<Pose2d> poseSupplier = null;

    /**
     * Provide the robot's current pose so it is drawn on the dashboard field view
     * even when no RoadRunner action is running.
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        if (!done()) {
            roadRunnerAction.preview(canvas);
            if (!roadRunnerAction.run(packet)) {
                roadRunnerAction = null;
            }
        } else if (poseSupplier != null) {
            Drawing.drawRobot(canvas, poseSupplier.get());
        } else {
            // Nothing to draw; the dashboard keeps showing its last field overlay.
            return;
        }

        dash.sendTelemetryPacket(packet);
    }

    public void drive(Action action) {
        if (!done()) {
            throw new RuntimeException("Drive action already in progress.");
        } else {
            roadRunnerAction = action;
        }
    }

    public void driveOverride(Action action) {
        roadRunnerAction = action;
    }

    public boolean done() {
        return roadRunnerAction == null;
    }

    public void cancel() {
        roadRunnerAction = null;
    }
}
