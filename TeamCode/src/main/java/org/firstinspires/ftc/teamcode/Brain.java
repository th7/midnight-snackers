package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import org.firstinspires.ftc.teamcode.base.SuperSystem;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Brain extends SuperSystem {
    private boolean usingCameraLocalization;
    private boolean turnTableToZeroMode = false;
    private boolean turnTableDebugOverride = false;
    private final PlanRunner planRunner = add(new PlanRunner());

    /** The camera may place the robot on the field only when playing for an alliance. */
    @Override
    public void init() {
        usingCameraLocalization = robot.alliance.usesCameraLocalization();
    }

    @Override
    protected void onLoop() {
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
        planRunner.cancel();
    }

    public void autoShootFast() {
        if (planRunner.done()) {
            planRunner.run(autoShootFastPlan());
        }
    }

    public void autoShootSlow() {
        if (planRunner.done()) {
            planRunner.run(autoShootSlowPlan());
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

    /** Whether the camera's tag sightings are used to place the robot on the field. */
    public boolean usingCameraLocalization() {
        return usingCameraLocalization;
    }

    public void setTurnTableDebugOverrideModeOn() {
        turnTableDebugOverride = true;
    }
}
