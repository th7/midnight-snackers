package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Use Alliance.blue or Alliance.red to get an instance of this class.
 * Use positive y coordinates to move toward your goal.
 * Use positive headings to turn toward your goal.
 */
public class Alliance {
    public static final Alliance blue = new Alliance(1, 1);
    public static final Alliance red = new Alliance(-1, -1);

    private final int headingSign;
    private final int ySign;

    private final int goalX;
    private final int goalY;

    private final int baseBackX;
    private final int baseBackY;

    Alliance(int headingSign, int ySign) {
        this.headingSign = headingSign;
        this.ySign = ySign;
        this.goalX = 60;
        this.goalY = 60 * ySign;
        this.baseBackX = -54;
        this.baseBackY = 24 * ySign;
    }

    public Pose2d pose(double x, double y, double heading) {
        return new Pose2d(x, y * this.ySign, heading * this.headingSign);
    }

    public Pose2d poseFacingGoal(int x, int y) {
        double heading = Math.atan2(this.goalY - y * this.ySign, this.goalX - x);
        return new Pose2d(x, y * this.ySign, heading);
    }

    public Pose2d backOfBase() {
        return new Pose2d(this.baseBackX, this.baseBackY, Math.PI);
    }
}
