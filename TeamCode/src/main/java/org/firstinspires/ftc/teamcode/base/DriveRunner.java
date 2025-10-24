package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class DriveRunner {
    public final FtcDashboard dash = FtcDashboard.getInstance();
    private Action roadRunnerAction = null;

    public void loop() {
        if (done()) {
            return;
        }

        TelemetryPacket packet = new TelemetryPacket();
        roadRunnerAction.preview(packet.fieldOverlay());
        if (!roadRunnerAction.run(packet)) {
            roadRunnerAction = null;
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

    public boolean done() {
        return roadRunnerAction == null;
    }
}
