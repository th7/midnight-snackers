package org.firstinspires.ftc.teamcode.base;

import java.util.LinkedHashSet;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The driver station's telemetry, divided into <b>channels</b> a driver turns on and off by the
 * name of whoever is printing. Telemetry costs a loop and a crowded screen, so a channel is off
 * until someone asks for it.
 *
 * <p>Whether anyone is reading is decided here and nowhere else. Each thing that prints is handed
 * its own channel when it is built and prints to it unconditionally, so a subsystem carries no
 * flag, answers no question about whether to print, and cannot print under somebody else's name:
 * the channel puts its own name on every caption. A driver's button reaches {@link #toggle}, which
 * is the one place that knows a channel can be off.
 */
public final class Channels implements Prints {
    private final Telemetry telemetry;
    private final Set<String> on = new LinkedHashSet<>();

    public Channels(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /** Somewhere for {@code name} to print, which reaches the screen only while it is on. */
    public Prints channel(String name) {
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

    /** Turns each named channel on if it is off, and off if it is on. */
    public void toggle(String... names) {
        for (String name : names) {
            if (!on.remove(name)) {
                on.add(name);
            }
        }
    }

    /** Whether {@code name} is reaching the screen. */
    public boolean isOn(String name) {
        return on.contains(name);
    }

    /** The channels reaching the screen, in the order they were turned on. */
    public Set<String> areOn() {
        return Set.copyOf(on);
    }

    /** Printing straight to the driver station, under nobody's channel: an op mode's own lines. */
    @Override
    public void addData(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    @Override
    public void addData(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
    }
}
