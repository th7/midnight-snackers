package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Where field drawings and mirrored telemetry go. On the robot this is the FTC Dashboard;
 * in tests and simulation it is whatever the test wants to inspect.
 */
public interface Dashboard {
    Telemetry telemetry();

    void send(TelemetryPacket packet);

    /**
     * The real FTC Dashboard. Only valid on the robot controller, where the dashboard has been started.
     */
    static Dashboard ftc() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        return new Dashboard() {
            @Override
            public Telemetry telemetry() {
                return dashboard.getTelemetry();
            }

            @Override
            public void send(TelemetryPacket packet) {
                dashboard.sendTelemetryPacket(packet);
            }
        };
    }
}
