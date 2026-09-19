package org.firstinspires.ftc.teamcode;

import java.util.Optional;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Brain implements Loopable {
    public static final String CHANNEL = "Brain";

    private final Prints telemetry;

    private final Drive drive;

    private final Launcher launcher;
    private final Camera camera;
    private final Nav nav;
    private final Turntable turntable;
    private final Alliance alliance;

    public Brain(
            Drive drive,
            Launcher launcher,
            Camera camera,
            Nav nav,
            Turntable turntable,
            Alliance alliance,
            Prints telemetry) {
        this.drive = drive;
        this.launcher = launcher;
        this.camera = camera;
        this.nav = nav;
        this.turntable = turntable;
        this.alliance = alliance;
        this.telemetry = telemetry;
        this.usingCameraLocalization = alliance.usesCameraLocalization();
    }

    private boolean usingCameraLocalization;
    private final PlanRunner planRunner = new PlanRunner();

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
                () -> nav.launchPose().map(drive::toward).orElse(true));
    }

    private Step launch() {
        return new Step("launch", launcher::launchyLaunch, launcher::launchDone);
    }

    private Step launchSlow() {
        return new Step("launchSlow", launcher::slowLaunchyLaunch, launcher::launchDone);
    }

    private void setTelemetry() {
        telemetry.addData("usingCameraLocalization", usingCameraLocalization);
        telemetry.addData("brainStep", planRunner.currentStep());
    }

    public void toggleCameraLocalization() {
        usingCameraLocalization = !usingCameraLocalization;
    }

    public boolean usingCameraLocalization() {
        return usingCameraLocalization;
    }

    @Override
    public void loop() {
        planRunner.loop();

        double relativeHeadingToTarget = nav.relativeHeadingToTarget();
        telemetry.addData("relativeHeadingToTarget", relativeHeadingToTarget);
        turntable.aimAt(relativeHeadingToTarget);

        Optional<Nav.Pose> sighting = camera.sighting();

        if (sighting.isPresent()) {
            telemetry.addData("camera pose found", true);

            Nav.Pose robotPose = sighting.get().rotated(-turntable.offsetRadians());

            if (usingCameraLocalization) {
                nav.sighted(robotPose);
            }
        }

        setTelemetry();
    }
}
