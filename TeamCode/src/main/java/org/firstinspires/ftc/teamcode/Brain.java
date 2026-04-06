package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.SuperSystem;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Brain extends SuperSystem {
    private boolean usingCameraLocalization = true;
    private boolean turnTableToZeroMode = false;
    private boolean turnTableDebugOverride = false;
    private Plan currentPlan = null;

    public Brain(ElapsedTime runtime, Telemetry telemetry, Launcher launcher, Drive drive, Camera camera, Nav nav, Turntable turntable) {
        super(runtime, telemetry, launcher, drive, camera, nav, turntable);
    }

    public void loop() {
        if (currentPlan != null && currentPlan.done()) {
            currentPlan = null;
        }

        if (!turnTableDebugOverride) {
            if (turnTableToZeroMode) {
                turntable.setTurnTablePosition(0);
            } else {
                turnTurnTableToTarget();
            }
        }

        Pose2d rawRoadrunnerPose = camera.calculateRoadrunnerPose();

        if (rawRoadrunnerPose != null) {
            telemetry.addData("camera pose found", true);
            Rotation2d turnTableOffset = Rotation2d.exp(turntable.getTurnTableOffsetRadians());
            Pose2d adjustedRoadrunnerPose = new Pose2d(rawRoadrunnerPose.position, rawRoadrunnerPose.heading.minus(turnTableOffset));

            if (usingCameraLocalization) {
                nav.setFieldPosition(adjustedRoadrunnerPose);
            }
        }

        setTelemetry();
    }

    public void turnTableToTargetModeOn() {
        turnTableDebugOverride = false;
        turnTableToZeroMode = false;
    }

    public void turnTurnTableToTarget() {
        double relativeHeadingToTarget = nav.relativeHeadingToTarget();
        telemetry.addData("relativeHeadingToTarget", relativeHeadingToTarget);
        turntable.setTurnTablePosition(relativeHeadingToTarget);
    }

    public void turnTableToZeroModeOn() {
        turnTableDebugOverride = false;
        turnTableToZeroMode = true;
    }

    public void cancelPlan() {
        currentPlan = null;
    }

    public void autoShootFast() {
        if (currentPlan == null) {
            currentPlan = autoShootFastPlan();
        }
    }

    public void autoShootSlow() {
        if (currentPlan == null) {
            currentPlan = autoShootSlowPlan();
        }
    }

    private Plan autoShootFastPlan() {
        return new Plan(
                moveToLaunchPose(),
                launch(),
                launch(),
                launch()
        );
    }

    private Plan autoShootSlowPlan() {
        return new Plan(
                moveToLaunchPose(),
                launchSlow(),
                launchSlow(),
                launchSlow()
        );
    }

    private Step moveToLaunchPose() {
        return new Step(
                "moveToLaunchPose",
                () -> {},
                () -> drive.fastDriveTo(nav.launchPose(), nav.currentPose())
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
