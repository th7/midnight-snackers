package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.base.Auto;
import org.firstinspires.ftc.teamcode.base.SuperSystem;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Plans extends SuperSystem {

    @Auto(alliance = Alliance.RELATIVE)
    public Plan scoreAThing() {
        return new Plan(
                backFromZeroALittle(),
                launchAll()
        );
    }

    @Auto(alliance = Alliance.RELATIVE)
    public Plan spinnyThing() {
        return new Plan(
                spin360(),
                move1FootForward(),
                move6InchesLeft(),
                moveBackTo0_0()
        );
    }

    private PlanPart move1FootForward() {
        return new Step(
                "move1FootForward",
                () -> drive.to(nav.strafeTo(12, 0, 0)),
                () -> {
                    if (nav.closeTo(12, 0, 0)) {
                        drive.cancel();
                        return true;
                    };
                    return false;
                }
        );
    }

    private PlanPart move6InchesLeft() {
        return new Step(
                "move6InchesLeft",
                () -> drive.to(nav.strafeTo(12, -6, 0)),
                () -> {
                    if (nav.closeTo(12, -6, 0)) {
                        drive.cancel();
                        return true;
                    };
                    return false;
                }
        );
    }

    private PlanPart moveBackTo0_0() {
        return new Step(
                "moveBackTo0_0",
                () -> drive.to(nav.strafeTo(0, 0, 0)),
                () -> {
                    if (nav.closeTo(0, 0, 0)) {
                        drive.cancel();
                        return true;
                    };
                    return false;
                }
        );
    }

    private Plan spin360() {
        return new Plan(
                turnLeft(),
                turnBackward(),
                turnRight(),
                turnForward()
        );
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
        return new Step(
                String.format("turnToHeadingAtZero %s", heading),
                () -> drive.to(nav.strafeTo(x, y, heading)),
                () -> {
                    if (nav.closeTo(x, y, heading)) {
                        drive.cancel();
                        return true;
                    };
                    return false;
                }
        );
    }

    @Auto(name = "BlueScoreAThingFromBack", alliance = Alliance.BLUE)
    @Auto(name = "RedScoreAThingFromBack", alliance = Alliance.RED)
    public Plan scoreAThingFromBack() {
        return new Plan(
                setFarLaunchPosition(),
                moveToBackWall(),
                moveToBackWallScorePosition(),
                launchAll(),
                toLoadingZone()
        );
    }

    private PlanPart moveToBackWall() {
        return driveTo(60, 12, 0);
    }

    private PlanPart moveToBackWallScorePosition() {
        return driveTo(60, 12, 0);
        //not correct pose
    }

    private PlanPart driveTo(double x, double y, double heading) {
        return new Step(
                String.format("driveTo %s, %s, %s, ", x, y, heading),
                () -> drive.to(nav.strafeTo(x, y, heading)),
                drive::done
        );
    }

    private Step launch() {
        return new Step(
                "launch",
                launcher::launchyLaunch,
                launcher::launchDone
        );
    }

    private Plan launchAll() {
        return new Plan(
                launch(),
                launch(),
                launch()
        );
    }

    //should be placed against the left side of the tile with the small launch line and against the wall
    private Step setFarLaunchPosition() {
        return new Step(
                "setBackPosition",
                () -> nav.setPose(nav.pose(-63.5, 15.375, 0)),
                () -> true
        );
    }

    private Step backFromZeroALittle() {
        return new Step(
                "backFromZeroALittle",
                () -> drive.to(nav.backwardTo(-20, 0, 0)),
                drive::done
        );
    }

    private Step toLoadingZone() {
        return new Step(
                "toLoadingZone",
                () -> drive.to(nav.backwardTo(-36, 12, 0)),
                drive::done
        );
    }

    @Auto(alliance = Alliance.RELATIVE)
    public PlanPart forwardLeftBackwardRight() {
        return new Plan(
                Step.waitFor("forwardLeftBackwardRight", 5),
                new Step(
                        "forwardLeftBackwardRight",
                        () -> drive.to(nav.strafePath(
                                nav.pose(24, 0, 0),
                                nav.pose(24, 24, 0),
                                nav.pose(0, 24, 0),
                                nav.pose(0, 0, 0)
                        )),
                        drive::done
                )
        );
    }

    @Auto(alliance = Alliance.RELATIVE)
    public PlanPart driveForward() {
        return new Plan(
                new Step(
                        "driveForward",
                        () -> drive.to(nav.strafeTo(24, 0, 0)),
                        drive::done
                )
        );
    }
}
