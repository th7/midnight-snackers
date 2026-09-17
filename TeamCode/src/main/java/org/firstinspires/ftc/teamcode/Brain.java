package org.firstinspires.ftc.teamcode;

import java.util.Optional;
import org.firstinspires.ftc.teamcode.base.Alliance;
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
    private final Alliance alliance;

    public Brain(Drive drive, Launcher launcher, Camera camera, Nav nav, Turntable turntable, Alliance alliance) {
        this.drive = drive;
        this.launcher = launcher;
        this.camera = camera;
        this.nav = nav;
        this.turntable = turntable;
        this.alliance = alliance;
    }

    private boolean usingCameraLocalization;
    private final PlanRunner planRunner = new PlanRunner();

    /** The camera may place the robot on the field only when playing for an alliance. */
    @Override
    protected void onInit() {
        usingCameraLocalization = alliance.usesCameraLocalization();
    }

    @Override
    protected void onTelemetry() {}

    @Override
    protected void onLoop() {
        planRunner.loop();

        // The standing request, every tick. Whether the turntable acts on it is the turntable's:
        // it may be parked or in the driver's hand, and the brain has no business knowing which.
        double relativeHeadingToTarget = nav.relativeHeadingToTarget();
        telemetry.addData("relativeHeadingToTarget", relativeHeadingToTarget);
        turntable.aimAt(relativeHeadingToTarget);

        Optional<Nav.Pose> sighting = camera.sighting();

        if (sighting.isPresent()) {
            telemetry.addData("camera pose found", true);
            // the camera faces where the turntable does, so the robot's heading is that less the turn
            Nav.Pose robotPose = sighting.get().rotated(-turntable.offsetRadians());

            if (usingCameraLocalization) {
                nav.sighted(robotPose);
            }
        }

        setTelemetry();
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
}
