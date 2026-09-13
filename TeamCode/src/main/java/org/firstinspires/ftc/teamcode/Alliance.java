package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;

/**
 * Which side of the field an op mode plays for, and so which way its {@link Nav} maps the field.
 */
public enum Alliance {
    BLUE(1, 1, Goal.BLUE),
    RED(-1, -1, Goal.RED),
    /**
     * No alliance: the field is wherever the robot started, headings and y run the blue way, and
     * the camera is not used to place the robot on the field.
     */
    RELATIVE(1, 1, null);

    /** What a heading given the blue way is multiplied by to play for this alliance. */
    public final int headingSign;
    /** What a y coordinate given the blue way is multiplied by to play for this alliance. */
    public final int ySign;
    /** Where this alliance's goal is on the field; null when playing for no alliance. */
    public final Vector2d launchTarget;

    Alliance(int headingSign, int ySign, Vector2d launchTarget) {
        this.headingSign = headingSign;
        this.ySign = ySign;
        this.launchTarget = launchTarget;
    }

    /** Whether the camera's tag sightings may place the robot on the field. */
    public boolean usesCameraLocalization() {
        return this != RELATIVE;
    }

    /** The goals, in RoadRunner coordinates: +x forward, +y left, facing the field from the audience. */
    private static final class Goal {
        private static final double BLUE_APRIL_TAG_X = 58.3;
        private static final double BLUE_APRIL_TAG_Y = 55.6;
        private static final double BLUE_X = BLUE_APRIL_TAG_X + 9;
        private static final double BLUE_Y = BLUE_APRIL_TAG_Y + 9;
        static final Vector2d BLUE = new Vector2d(BLUE_X, BLUE_Y);
        static final Vector2d RED = new Vector2d(BLUE_X, -BLUE_Y);
    }
}
