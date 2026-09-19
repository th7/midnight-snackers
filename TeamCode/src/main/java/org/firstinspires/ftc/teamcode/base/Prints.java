package org.firstinspires.ftc.teamcode.base;

public interface Prints {
    Prints NOWHERE = new Prints() {
        @Override
        public void addData(String caption, Object value) {}

        @Override
        public void addData(String caption, String format, Object... args) {}
    };

    void addData(String caption, Object value);

    void addData(String caption, String format, Object... args);
}
