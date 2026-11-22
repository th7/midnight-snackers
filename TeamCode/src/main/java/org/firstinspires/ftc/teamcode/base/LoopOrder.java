package org.firstinspires.ftc.teamcode.base;

import java.util.ArrayList;
import java.util.List;

public final class LoopOrder implements Loopable {
    private final List<Subsystem> subsystems = new ArrayList<>();

    void add(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    @Override
    public void loop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.loop();
        }
    }
}
