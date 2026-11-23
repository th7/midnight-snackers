package org.firstinspires.ftc.teamcode.base;

import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Channels implements Prints {
    private final Telemetry telemetry;
    private final TreeSet<String> made = new TreeSet<>();
    private Optional<String> showing = Optional.empty();

    public Channels(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Prints channel(String name) {
        made.add(name);
        return new Prints() {
            @Override
            public void addData(String caption, Object value) {
                if (isShowing(name)) {
                    telemetry.addData(name + "." + caption, value);
                }
            }

            @Override
            public void addData(String caption, String format, Object... args) {
                if (isShowing(name)) {
                    telemetry.addData(name + "." + caption, format, args);
                }
            }
        };
    }

    public void step() {
        showing = showing.isPresent()
                ? Optional.ofNullable(made.higher(showing.get()))
                : Optional.ofNullable(made.isEmpty() ? null : made.first());
    }

    public Optional<String> showing() {
        return showing;
    }

    public Set<String> made() {
        return Set.copyOf(made);
    }

    private boolean isShowing(String name) {
        return showing.isPresent() && showing.get().equals(name);
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
