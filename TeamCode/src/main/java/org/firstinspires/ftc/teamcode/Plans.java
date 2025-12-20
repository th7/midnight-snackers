package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Plans {
    public Drive drive;
    public Launcher launcher;
    public ElapsedTime runtime;
    private double startedWaitAt;
    private double timeoutStartedAt;
    private Motif motif;
    public Plans(Drive drive, Launcher launcher, ElapsedTime runtime) {
        this.drive = drive;
        this.launcher = launcher;
        this.runtime = runtime;
    }

    public Plan redScoreAThingFromBack() {
        return scoreAThingFromBack(Alliance.red);
    }

    public Plan blueScoreAThingFromBack() {
        return scoreAThingFromBack(Alliance.blue);
    }

    public Plan scoreAThing() {
        return new Plan(
                setZeroPosition(),
                setCloseLaunchPower(),
                backFromZeroALittle(),
                launchAll()
        );
    }

    public Plan scoreAThingFromBack(Alliance alliance) {
        return new Plan(
                setBackPosition(alliance),
                setCloseLaunchPower(),
                moveToScorePosition(alliance),
                launchAll(),
                toLoadingZone(alliance)
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
                setZeroPosition(),
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
                    startedWaitAt = runtime.time();
                },
                () -> {
                    return runtime.time() >= startedWaitAt + seconds;
                }
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
                drive::splineSquiggleSquare,
                drive::done
        );
    }

    private Step toZeroPosition() {
        return new Step(
                "toZeroPosition",
                drive::toZeroPosition,
                drive::done
        );
    }

    private Step setZeroPosition() {
        return new Step(
                "setZeroPosition",
                () -> drive.setPose(new Pose2d(0, 0, 0)),
                () -> true
        );
    }

    //should be placed against the left side of the tile with the small launch line and against the wall
    private Step setBackPosition(Alliance alliance) {
        return new Step(
                "setBackPosition",
                () -> drive.setPose(alliance.pose(-63.5, 15.375, 0)),
                () -> true
        );
    }

    private Step moveToScorePosition(Alliance alliance) {
        return new Step(
                "moveToScorePosition",
                () -> drive.drivePathForward(alliance.pose(33, 33, Math.PI / 4)),
                drive::done
        );
    }

    private Step moveFromBackToCloseGoal(Alliance alliance) {
        return new Step(
                "moveFromBackToCloseGoal",
                () -> drive.drivePathForward(
                        alliance.pose(12, 12, Math.PI / 4),
                        alliance.pose(48, 48, Math.PI / 4)
                ),
                drive::done
        );
    }

    private Step getOffLaunchLine(Alliance alliance) {
        return new Step(
                "getOffLaunchLine",
                () -> drive.drivePathForward(
                        alliance.pose(24, 24, Math.PI / 4),
                        alliance.pose(0, 24, Math.PI / 4)
                ),
                drive::done
        );
    }

    private Step backFromZeroALittle() {
        return new Step(
                "backFromZeroALittle",
                () -> drive.drivePathBackward(
                        new Pose2d(-20, 0, 0)
                ),
                drive::done
        );
    }

    private Step toLoadingZone(Alliance alliance) {
        return new Step(
                "toLoadingZone",
                () -> drive.drivePathBackward(
                        alliance.pose(-36, 12, 0)
                ),
                drive::done
        );
    }

    private Step detectMotif() {
        return new Step(
                "detectMotif",
                () -> {
                    timeoutStartedAt = runtime.time();
                },
                () -> {
                    motif = drive.motif();
                    return motif != null || runtime.time() > timeoutStartedAt + 3;
                }
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
