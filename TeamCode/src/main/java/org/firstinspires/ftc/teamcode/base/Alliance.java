package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.roadrunner.Vector2d;

public enum Alliance {
    BLUE(1, 1, Goal.BLUE),
    RED(-1, -1, Goal.RED),

    RELATIVE(1, 1, null);

    public final int headingSign;

    public final int ySign;

    public final Vector2d launchTarget;

    Alliance(int headingSign, int ySign, Vector2d launchTarget) {
        this.headingSign = headingSign;
        this.ySign = ySign;
        this.launchTarget = launchTarget;
    }

    public boolean usesCameraLocalization() {
        return this != RELATIVE;
    }

    private static final class Goal {
        private static final double BLUE_APRIL_TAG_X = 58.3;
        private static final double BLUE_APRIL_TAG_Y = 55.6;
        private static final double BLUE_X = BLUE_APRIL_TAG_X + 9;
        private static final double BLUE_Y = BLUE_APRIL_TAG_Y + 9;
        static final Vector2d BLUE = new Vector2d(BLUE_X, BLUE_Y);
        static final Vector2d RED = new Vector2d(BLUE_X, -BLUE_Y);
    }
}
