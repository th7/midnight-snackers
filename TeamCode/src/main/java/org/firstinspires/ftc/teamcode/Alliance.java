package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * Which side of the field an op mode plays for, and so which way its {@link Nav} maps the field.
 */
public enum Alliance {
    BLUE,
    RED,
    /**
     * No alliance: the field is wherever the robot started, headings and y run the blue way, and
     * the camera is not used to place the robot on the field.
     */
    RELATIVE;

    public Nav nav(MecanumDrive mecanumDrive, ElapsedTime runtime, Telemetry telemetry) {
        switch (this) {
            case BLUE:
                return Nav.blue(mecanumDrive, runtime, telemetry);
            case RED:
                return Nav.red(mecanumDrive, runtime, telemetry);
            default:
                return Nav.relative(mecanumDrive, runtime, telemetry);
        }
    }

    /** Whether the camera's tag sightings may place the robot on the field. */
    public boolean usesCameraLocalization() {
        return this != RELATIVE;
    }
}
