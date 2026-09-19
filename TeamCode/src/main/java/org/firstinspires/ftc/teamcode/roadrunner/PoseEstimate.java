package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import java.util.List;

public interface PoseEstimate {
    Pose2d pose();

    PoseVelocity2d velocity();

    List<Pose2d> trail();
}
