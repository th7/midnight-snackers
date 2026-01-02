package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.SuperSystem;

public class Brain extends SuperSystem {

    private boolean usingCameraLocalization = true;
    private int cameraLocalizationDroppedDueToMovement = 0;

    public Brain(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
        super.init();
        telemetry.addData("Brain.init()", true);
    }

    public void loop() {
        super.loop();
        setTelemetry();

        Pose2d roadrunnerPose = camera.calculateRoadrunnerPose();

        if (roadrunnerPose != null) {
            if (usingCameraLocalization && drive.nearlyStopped()) {
                drive.setFieldPosition(roadrunnerPose);
            }
            if (!drive.nearlyStopped()) {
                cameraLocalizationDroppedDueToMovement += 1;
            }
            telemetry.addData("roadrunnerPoseFound", true);
        }
    }

    public void autoShootSlowBlue() {
        if (drive.turnToBlue()) {
            if (drive.driveToBlue()) {
                launcher.slowLaunchyLaunch();
            }
        }
    }

    public void autoShootFastBlue() {
        if (drive.turnToBlue()) {
            if (drive.driveToBlue()) {
                launcher.launchyLaunch();
            }
        }
    }

    private void setTelemetry() {
        telemetry.addData("usingCameraLocalization", usingCameraLocalization);
        telemetry.addData("cameraLocalizationDroppedDueToMovement", cameraLocalizationDroppedDueToMovement);
    }

    public void toggleCameraLocalization() {
        usingCameraLocalization = !usingCameraLocalization;
    }
}
