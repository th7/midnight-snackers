package org.firstinspires.ftc.teamcode.base;

/**
 * Somewhere to print: the whole of what anything on the robot needs of the driver station's
 * telemetry, which is two ways of naming a value. Nothing that prints needs to know whether anyone
 * is reading, so nothing that prints is told.
 */
public interface Prints {
    /**
     * Nowhere: it goes to no screen. For code that runs where none of our telemetry is read --
     * Road Runner's tuning op modes, which drive its own dashboard -- said out loud at the call
     * site rather than left as a null to find.
     */
    Prints NOWHERE = new Prints() {
        @Override
        public void addData(String caption, Object value) {}

        @Override
        public void addData(String caption, String format, Object... args) {}
    };

    void addData(String caption, Object value);

    void addData(String caption, String format, Object... args);
}
