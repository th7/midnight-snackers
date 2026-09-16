package org.firstinspires.ftc.teamcode;

import java.util.Optional;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Brain extends SubSystem {
    /** The subsystems the brain coordinates. */
    private final Drive drive;

    private final Launcher launcher;
    private final Camera camera;
    private final Nav nav;
    private final Turntable turntable;

    public Brain(Drive drive, Launcher launcher, Camera camera, Nav nav, Turntable turntable) {
        this.drive = drive;
        this.launcher = launcher;
        this.camera = camera;
        this.nav = nav;
        this.turntable = turntable;
    }

    private boolean usingCameraLocalization;
    private boolean turnTableToZeroMode = false;
    private boolean turnTableDebugOverride = false;
    private final PlanRunner planRunner = add(new PlanRunner());

    /** The camera may place the robot on the field only when playing for an alliance. */
    @Override
    protected void onInit() {
        usingCameraLocalization = robot.alliance.usesCameraLocalization();
    }

    @Override
    protected void onTelemetry() {}

    @Override
    protected void onLoop() {
        if (!turnTableDebugOverride) {
            if (turnTableToZeroMode) {
                turntable.setTurnTablePosition(0);
            } else {
                turnTurnTableToTarget();
            }
        }

        Optional<Nav.Pose> sighting = camera.sighting();

        if (sighting.isPresent()) {
            telemetry.addData("camera pose found", true);
            // the camera faces where the turntable does, so the robot's heading is that less the turn
            Nav.Pose robotPose = sighting.get().rotated(-turntable.getTurnTableOffsetRadians());

            if (usingCameraLocalization) {
                nav.setFieldPosition(robotPose);
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
        return new Plan(moveToLaunchPose(), launch(), launch(), launch());
    }

    private Plan autoShootSlowPlan() {
        return new Plan(moveToLaunchPose(), launchSlow(), launchSlow(), launchSlow());
    }

    private Step moveToLaunchPose() {
        return new Step(
                "moveToLaunchPose",
                () -> {},
                () -> nav.launchPose().map(drive::toward).orElse(true) // nowhere to go without a goal
                );
    }

    private Step launch() {
        return new Step("launchSlow", launcher::launchyLaunch, launcher::launchDone);
    }

    private Step launchSlow() {
        return new Step("launchSlow", launcher::slowLaunchyLaunch, launcher::launchDone);
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
