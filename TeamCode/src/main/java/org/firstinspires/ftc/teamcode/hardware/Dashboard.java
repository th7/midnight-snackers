package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Dashboard {
    Telemetry telemetry();

    void send(TelemetryPacket packet);

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
