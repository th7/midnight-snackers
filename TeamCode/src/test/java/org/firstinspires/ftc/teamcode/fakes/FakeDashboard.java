package org.firstinspires.ftc.teamcode.fakes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Dashboard;

public class FakeDashboard implements Dashboard {
    public final List<TelemetryPacket> packets = new ArrayList<>();
    private final Telemetry telemetry = new FakeTelemetry();

    @Override
    public Telemetry telemetry() {
        return telemetry;
    }

    @Override
    public void send(TelemetryPacket packet) {
        packets.add(packet);
    }
}
