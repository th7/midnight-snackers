package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.SuperSystem;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Brain extends SuperSystem {

    private boolean usingCameraLocalization = true;
    private boolean turnTableToZeroMode = false;
    private boolean turnTableDebugOverride = false;
    private int cameraLocalizationDroppedDueToMovement = 0;
    private Plan currentPlan = null;
    private Vector2d launchTarget = null;

    public Brain(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
        super.init();
        telemetry.addData("Brain.init()", true);
    }

    public void loop() {
        super.loop();

        if (currentPlan != null && currentPlan.done()) {
            currentPlan = null;
        }

        if (!turnTableDebugOverride) {
            if (turnTableToZeroMode) {
                launcher.setTurnTablePosition(0);
            } else {
                turnTurnTableToTarget(launchTarget);
            }
        }

        Pose2d rawRoadrunnerPose = camera.calculateRoadrunnerPose();

        if (rawRoadrunnerPose != null) {
            Rotation2d turnTableOffset = Rotation2d.exp(launcher.getTurnTableOffsetRadians());
            Pose2d adjustedRoadrunnerPose = new Pose2d(rawRoadrunnerPose.position, rawRoadrunnerPose.heading.minus(turnTableOffset));

            if (usingCameraLocalization && drive.nearlyStopped()) {
                drive.setFieldPosition(adjustedRoadrunnerPose);
            }
            if (!drive.nearlyStopped()) {
                cameraLocalizationDroppedDueToMovement += 1;
            }
            telemetry.addData("roadrunnerPoseFound", true);
        }

        setTelemetry();
    }

    public void setLaunchTarget(Vector2d launchTarget) {
        this.launchTarget = launchTarget;
    }

    public void turnTableToTargetModeOn() {
        turnTableDebugOverride = false;
        turnTableToZeroMode = false;
    }

    public void turnTurnTableToTarget(Vector2d launchTarget) {
        double relativeHeadingToTarget = drive.relativeHeadingToTarget(launchTarget);
        telemetry.addData("relativeHeadingToTarget", relativeHeadingToTarget);
        launcher.setTurnTablePosition(relativeHeadingToTarget);
    }

    public void turnTableToZeroModeOn() {
        turnTableDebugOverride = false;
        turnTableToZeroMode = true;
    }

    public void cancelPlan() {
        currentPlan = null;
    }

    public void autoShootFast(Vector2d launchTarget) {
        if (currentPlan == null) {
            currentPlan = autoShootFastPlan(launchTarget);
        }
    }

    public void autoShootSlow(Vector2d launchTarget) {
        if (currentPlan == null) {
            currentPlan = autoShootSlowPlan(launchTarget);
        }
    }

    private Plan autoShootFastPlan(Vector2d launchTarget) {
        return new Plan(
                moveToLaunchPose(launchTarget),
                launch(),
                launch(),
                launch()
        );
    }

    private Plan autoShootSlowPlan(Vector2d launchTarget) {
        return new Plan(
                moveToLaunchPose(launchTarget),
                launchSlow(),
                launchSlow(),
                launchSlow()
        );
    }

    private Step moveToLaunchPose(Vector2d launchTarget) {
        return new Step(
                "moveToLaunchPose",
                () -> {},
                () -> drive.fastDriveToLaunchPose(launchTarget)
        );
    }

    private Step launch() {
        return new Step(
                "launchSlow",
                launcher::launchyLaunch,
                launcher::launchDone
        );
    }

    private Step launchSlow() {
        return new Step(
                "launchSlow",
                launcher::slowLaunchyLaunch,
                launcher::launchDone
        );
    }

    private void setTelemetry() {
        telemetry.addData("usingCameraLocalization", usingCameraLocalization);
        telemetry.addData("cameraLocalizationDroppedDueToMovement", cameraLocalizationDroppedDueToMovement);
    }

    public void toggleCameraLocalization() {
        usingCameraLocalization = !usingCameraLocalization;
    }

    public void enableCameraLocalization() {
        usingCameraLocalization = true;
    }

    public void disableCameraLocalization() {
        usingCameraLocalization = false;
    }

    public void setTurnTableDebugOverrideModeOn() {
        turnTableDebugOverride = true;
    }
}
