package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import java.util.List;

/**
 * Where the robot is, how fast it is going, and where it has been: everything a trajectory being
 * followed needs to know about the robot's own motion.
 *
 * <p>It is an interface so that this package can ask for those three things without knowing what
 * keeps them. What keeps them is the robot's localizer, which reads the dead wheels and the IMU
 * once a loop; this package reads them and never updates them.
 */
public interface PoseEstimate {
    /** Where the robot believes it is, as of this loop's update. */
    Pose2d pose();

    /** How fast it was going at that update, in its own frame. */
    PoseVelocity2d velocity();

    /** Where it has recently been, oldest first, for the trail drawn on the dashboard. */
    List<Pose2d> trail();
}
