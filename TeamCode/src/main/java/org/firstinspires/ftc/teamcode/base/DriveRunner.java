package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;

public class DriveRunner implements Loopable {
    private final Dashboard dashboard;
    private final Supplier<Pose2d> poseSupplier;
    private Action roadRunnerAction = null;

    /**
     * @param poseSupplier the robot's current pose, so it is drawn on the dashboard field view even
     *                     when no RoadRunner action is running; null to draw nothing then
     */
    public DriveRunner(Dashboard dashboard, Supplier<Pose2d> poseSupplier) {
        this.dashboard = dashboard;
        this.poseSupplier = poseSupplier;
    }

    @Override
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

        dashboard.send(packet);
    }

    public void drive(Action action) {
        if (!done()) {
            throw new IllegalStateException("an action is already in progress");
        } else {
            roadRunnerAction = action;
        }
    }

    public boolean done() {
        return roadRunnerAction == null;
    }

    public void cancel() {
        roadRunnerAction = null;
    }
}
