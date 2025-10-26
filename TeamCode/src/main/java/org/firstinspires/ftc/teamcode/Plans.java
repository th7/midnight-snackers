package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Plans {
    enum Motif {
        GPP,
        PGP,
        PPG;
    }
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

    public Plan redFarScoreAThing() {
        return farScoreAThing(Alliance.red);
    }

    public Plan redCloseScoreAThing() {
        return nearScoreAThing(Alliance.red);
    }

    public Plan blueFarScoreAThing() {
        return farScoreAThing(Alliance.blue);
    }

    public Plan blueCloseScoreAThing() {
        return nearScoreAThing(Alliance.blue);
    }

    public Plan scoreAThing() {
        return new Plan(
                setZeroPosition(),
                setCloseLaunchPower(),
                waitFor(4),
                waitForFlywheel(),
                launchStep(),
                backFromZeroALittle()

        );
    }

    public Plan scoreAMotif() {
        return new Plan(
                detectMotif(),
                launchMotif()
        );
    }

    public Plan launchMotif() {
        return new Plan(
                launchMotifFirst(),
                launchMotifSecond(),
                launchMotifThird()
        );
    }

    private Plan farScoreAThing(Alliance alliance) {
        return new Plan(
                setBackPosition(alliance),
                setFarLaunchPowerStep(),
                moveToFarScorePosition(alliance),
                waitForFlywheel(),
                launchStep(),
                getOffLaunchLine(alliance)

        );
    }

    public Plan nearScoreAThing(Alliance alliance) {
        return new Plan(
                setBackPosition(alliance),
                setCloseLaunchPower(),
                moveFromBackToCloseGoal(alliance),
                waitForFlywheel(),
                launchStep(),
                getOffLaunchLine(alliance)
        );
    }

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

    private Step waitForFlywheel() {
        return new Step(
                "waitForFlywheel",
                () -> {
                },
                launcher::flywheelReady
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

    private Step launchStep() {
        return new Step(
                "launch",
                launcher::launch,
                launcher::done
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

    private Step moveToFarScorePosition(Alliance alliance) {
        return new Step(
                "moveToFarScorePosition",
                () -> drive.drivePath(alliance.pose(33, 33, Math.PI / 4)),
                drive::done
        );
    }

    private Step moveFromBackToCloseGoal(Alliance alliance) {
        return new Step(
                "moveFromBackToCloseGoal",
                () -> drive.drivePath(
                        alliance.pose(12, 12, Math.PI / 4),
                        alliance.pose(48, 48, Math.PI / 4)
                ),
                drive::done
        );
    }

    private Step getOffLaunchLine(Alliance alliance) {
        return new Step(
                "getOffLaunchLine",
                () -> drive.drivePath(
                        alliance.pose(24, 24, Math.PI / 4),
                        alliance.pose(0, 24, Math.PI / 4)
                ),
                drive::done
        );
    }

    private Step backFromZeroALittle() {
        return new Step(
                "backFromZeroALittle",
                () -> drive.drivePath(
                        new Pose2d(-12, 0, 0)
                ),
                drive::done
        );
    }

    private Step detectMotif() {
        return new Step(
                "detectMotif",
                () -> { timeoutStartedAt = runtime.time(); },
                () -> {
                    motif = drive.motif();
                    return motif != null || runtime.time() > timeoutStartedAt + 3;
                }
        );
    }

    private Step launchMotifFirst() {
        return new Step(
                "launchMotifFirst",
                () -> { launcher.launchMotifFirst(motif); },
                launcher::done
        );
    }

    private Step launchMotifSecond() {
        return new Step(
                "launchMotifSecond",
                () -> { launcher.launchMotifSecond(motif); },
                launcher::done
        );
    }

    private Step launchMotifThird() {
        return new Step(
                "launchMotifThird",
                () -> { launcher.launchMotifThird(motif); },
                launcher::done
        );
    }

//    private detectMotif() {
//        if (drive.motif()) {
//            motif = drive.motif();
//        }
//    }
}
