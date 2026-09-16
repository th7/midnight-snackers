package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.base.Auto;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Plans extends SubSystem {

    @Override
    protected void onInit() {}

    /** The plans are looked up when an op mode runs one; there is nothing to do each tick. */
    @Override
    protected void onLoop() {}

    @Override
    protected void onTelemetry() {}

    @Auto(alliance = Alliance.RELATIVE)
    public Plan scoreAThing() {
        return new Plan(backFromZeroALittle(), launchAll());
    }

    @Auto(alliance = Alliance.RELATIVE)
    public Plan spinnyThing() {
        return new Plan(spin360(), move1FootForward(), move6InchesLeft(), moveBackTo0_0());
    }

    private PlanPart move1FootForward() {
        return driveNear("move1FootForward", 12, 0, 0);
    }

    private PlanPart move6InchesLeft() {
        return driveNear("move6InchesLeft", 12, -6, 0);
    }

    private PlanPart moveBackTo0_0() {
        return driveNear("moveBackTo0_0", 0, 0, 0);
    }

    private Plan spin360() {
        return new Plan(turnLeft(), turnBackward(), turnRight(), turnForward());
    }

    private Step turnRight() {
        return turnToHeadingAtZero(0, 1, Math.PI / 2);
    }

    private Step turnForward() {
        return turnToHeadingAtZero(0, 0, 0);
    }

    private Step turnLeft() {
        return turnToHeadingAtZero(0, -1, -Math.PI / 2);
    }

    private Step turnBackward() {
        return turnToHeadingAtZero(-1, 0, -Math.PI);
    }

    private Step turnToHeadingAtZero(double x, double y, double heading) {
        return driveNear(String.format("turnToHeadingAtZero %s", heading), x, y, heading);
    }

    /**
     * Strafes toward the pose and is done as soon as the robot is near it, without waiting for
     * Road Runner to settle; the rest of the path is cancelled.
     */
    private Step driveNear(String name, double x, double y, double heading) {
        return new Step(name, () -> robot.drive.strafeTo(robot.nav.pose(x, y, heading)), () -> {
            if (robot.nav.near(robot.nav.pose(x, y, heading))) {
                robot.drive.cancel();
                return true;
            }
            return false;
        });
    }

    @Auto(name = "BlueScoreAThingFromBack", alliance = Alliance.BLUE)
    @Auto(name = "RedScoreAThingFromBack", alliance = Alliance.RED)
    public Plan scoreAThingFromBack() {
        return new Plan(
                setFarLaunchPosition(), moveToBackWall(), moveToBackWallScorePosition(), launchAll(), toLoadingZone());
    }

    private PlanPart moveToBackWall() {
        return driveTo(60, 12, 0);
    }

    private PlanPart moveToBackWallScorePosition() {
        return driveTo(60, 12, 0);
        // not correct pose
    }

    private PlanPart driveTo(double x, double y, double heading) {
        return new Step(
                String.format("driveTo %s, %s, %s, ", x, y, heading),
                () -> robot.drive.strafeTo(robot.nav.pose(x, y, heading)),
                robot.drive::done);
    }

    private Step launch() {
        return new Step("launch", robot.launcher::launchyLaunch, robot.launcher::launchDone);
    }

    private Plan launchAll() {
        return new Plan(launch(), launch(), launch());
    }

    // should be placed against the left side of the tile with the small launch line and against the wall
    private Step setFarLaunchPosition() {
        return new Step("setBackPosition", () -> robot.nav.setPose(robot.nav.pose(-63.5, 15.375, 0)), () -> true);
    }

    private Step backFromZeroALittle() {
        return new Step(
                "backFromZeroALittle", () -> robot.drive.backwardTo(robot.nav.pose(-20, 0, 0)), robot.drive::done);
    }

    private Step toLoadingZone() {
        return new Step("toLoadingZone", () -> robot.drive.backwardTo(robot.nav.pose(-36, 12, 0)), robot.drive::done);
    }

    @Auto(alliance = Alliance.RELATIVE)
    public PlanPart forwardLeftBackwardRight() {
        return new Plan(
                Step.waitFor("forwardLeftBackwardRight", 5, robot.clock),
                new Step(
                        "forwardLeftBackwardRight",
                        () -> robot.drive.strafeTo(
                                robot.nav.pose(24, 0, 0),
                                robot.nav.pose(24, 24, 0),
                                robot.nav.pose(0, 24, 0),
                                robot.nav.pose(0, 0, 0)),
                        robot.drive::done));
    }

    @Auto(alliance = Alliance.RELATIVE)
    public PlanPart driveForward() {
        return new Plan(
                new Step("driveForward", () -> robot.drive.strafeTo(robot.nav.pose(24, 0, 0)), robot.drive::done));
    }
}
