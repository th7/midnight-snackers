package org.firstinspires.ftc.teamcode.base;

import java.util.LinkedHashSet;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Channels implements Prints {
    private final Telemetry telemetry;
    private final Set<String> on = new LinkedHashSet<>();
    private final Set<String> made = new LinkedHashSet<>();

    public Channels(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Prints channel(String name) {
        made.add(name);
        return new Prints() {
            @Override
            public void addData(String caption, Object value) {
                if (on.contains(name)) {
                    telemetry.addData(name + "." + caption, value);
                }
            }

            @Override
            public void addData(String caption, String format, Object... args) {
                if (on.contains(name)) {
                    telemetry.addData(name + "." + caption, format, args);
                }
            }
        };
    }

    public void toggle(String... names) {
        for (String name : names) {
            if (!on.remove(name)) {
                on.add(name);
            }
        }
    }

    public boolean isOn(String name) {
        return on.contains(name);
    }

    public Set<String> areOn() {
        return Set.copyOf(on);
    }

    public Set<String> made() {
        return Set.copyOf(made);
    }

    @Override
    public void addData(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    @Override
    public void addData(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
    }
}
