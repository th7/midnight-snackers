package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.SuperSystem;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Plans extends SuperSystem {
    private final Brain brain;
    private double startedWaitAt;
    private double timeoutStartedAt;
    private Motif motif;

    public Plans(ElapsedTime runtime, Telemetry telemetry, Launcher launcher, Drive drive, Camera camera, Nav nav, Turntable turntable, Brain brain) {
        super(runtime, telemetry, launcher, drive, camera, nav, turntable);
        this.brain = brain;
    }

    public Plan scoreAThing() {
        return new Plan(
                setCloseLaunchPower(),
                backFromZeroALittle(),
                launchAll()
        );
    }

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
                drive::done
        );
    }

    private PlanPart move6InchesLeft() {
        return new Step(
                "move6InchesLeft",
                () -> drive.to(nav.strafeTo(12, -6, 0)),
                drive::done
        );
    }

    private PlanPart moveBackTo0_0() {
        return new Step(
                "moveBackTo0_0",
                () -> drive.to(nav.strafeTo(0, 0, 0)),
                drive::done
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
                "turnToHeadingAtZero",
                () -> drive.to(nav.strafeTo(x, y, heading)),
                drive::done
        );
    }

    public Plan scoreAThingFromBack() {
        return new Plan(
                setFarLaunchPosition(),
                setCloseLaunchPower(),
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

//    public Plan scoreAMotif() {
//        return new Plan(
//                detectMotif(),
//                launchMotif()
//        );
//    }

//    public Plan launchMotif() {
//        return new Plan(
//                launchMotifFirst(),
//                launchMotifSecond(),
//                launchMotifThird()
//        );
//    }

    public Plan splineSquiggleSquare() {
        return new Plan(
                splineSquiggleSquareStep(),
                toZeroPosition()
        );
    }

    private Step setFarLaunchPowerStep() {
        return new Step(
                "setFarLaunchPower",
                launcher::setFarLaunchPower,
                () -> true
        );
    }

    private Step setCloseLaunchPower() {
        return new Step(
                "setCloseLaunchPower",
                launcher::setCloseLaunchPower,
                () -> true
        );
    }

    private Step waitFor(double seconds) {
        return new Step(
                "waitFor " + seconds,
                () -> {
                },
                Step.secondsElapsed(seconds)
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

    private Step splineSquiggleSquareStep() {
        return new Step(
                "splineSquiggleSquare",
                () -> drive.to(splineSquiggleSquarePath()),
                drive::done
        );
    }

    private Action splineSquiggleSquarePath() {
        return nav.forwardPath(
                nav.pose(72, 0, 0),
                nav.pose(96, 0, Math.PI / 4),
                nav.pose(96, 24, Math.PI * 3 / 4),
                nav.pose(72, 24, Math.PI * 1),
                nav.pose(48, 24, Math.PI * 1),
                nav.pose(24, 24, Math.PI * 3 / 4),
                nav.pose(24, 48, Math.PI * 3 / 4),
                nav.pose(0, 48, Math.PI * -3 / 4),
                nav.pose(0, 24, Math.PI / -2),
                nav.pose(0, 0, 0)
        );
    }

    private Step toZeroPosition() {
        return new Step(
                "toZeroPosition",
                () -> drive.to(nav.strafeTo(0, 0, 0)),
                drive::done
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

//    private Step moveToScorePosition() {
//        return new Step(
//                "moveToScorePosition",
//                () -> drive.to(nav.strafeTo(33, 33, Math.PI / 4)),
//                drive::done
//        );
//    }
//
//    private Step moveFromBackToCloseGoal() {
//        return new Step(
//                "moveFromBackToCloseGoal",
//                () -> drive.drivePathForward(
//                        nav.pose(12, 12, Math.PI / 4),
//                        nav.pose(48, 48, Math.PI / 4)
//                ),
//                drive::done
//        );
//    }

//    private Step getOffLaunchLine() {
//        return new Step(
//                "getOffLaunchLine",
//                () -> drive.drivePathForward(
//                        nav.pose(24, 24, Math.PI / 4),
//                        nav.pose(0, 24, Math.PI / 4)
//                ),
//                drive::done
//        );
//    }

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

//    private Step detectMotif() {
//        return new Step(
//                "detectMotif",
//                () -> {
//                    timeoutStartedAt = runtime.time();
//                },
//                () -> {
//                    motif = drive.motif();
//                    return motif != null || runtime.time() > timeoutStartedAt + 3;
//                }
//        );
//    }

    public PlanPart forwardLeftBackwardRight() {
        return new Plan(
                waitFor(5),
                new Step(
                        "forwardLeftBackwardRight",
                        () -> drive.to(forwardLeftBackwardRightAction()),
                        drive::done
                )
        );
    }

    public Action forwardLeftBackwardRightAction() {
        return nav.strafePath(
                nav.pose(24, 0, 0),
                nav.pose(24, 24, 0),
                nav.pose(0, 24, 0),
                nav.pose(0, 0, 0)
        );
    }

    public PlanPart driveForward() {
        return new Plan(
                new Step(
                        "driveForward",
                        () -> drive.to(nav.strafeTo(24, 0, 0)),
                        drive::done
                )
        );
    }

    enum Motif {
        GPP,
        PGP,
        PPG
    }

//    private Step launchMotifFirst() {
//        return new Step(
//                "launchMotifFirst",
//                () -> { launcher.launchMotifFirst(motif); },
//                launcher::gateLaunchDone
//        );
//    }
//
//    private Step launchMotifSecond() {
//        return new Step(
//                "launchMotifSecond",
//                () -> { launcher.launchMotifSecond(motif); },
//                launcher::gateLaunchDone
//        );
//    }
//
//    private Step launchMotifThird() {
//        return new Step(
//                "launchMotifThird",
//                () -> { launcher.launchMotifThird(motif); },
//                launcher::gateLaunchDone
//        );
//    }

//    private detectMotif() {
//        if (drive.motif()) {
//            motif = drive.motif();
//        }
//    }
}
