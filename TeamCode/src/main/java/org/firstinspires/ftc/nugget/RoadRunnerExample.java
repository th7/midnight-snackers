package org.firstinspires.ftc.nugget;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.function.UnaryOperator;
import org.firstinspires.ftc.teamcode.control.DriveRunner;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.planrunner.RunsAPlan;
import org.firstinspires.ftc.teamcode.planrunner.Step;

@Autonomous(name = "Nugget RoadRunner Example", group = "Nugget")
public class RoadRunnerExample extends NuggetOpMode implements RunsAPlan {
    public static final Pose2d START = new Pose2d(-24, -44, 0);

    private TankLocalizer localizer;
    private Trajectories trajectories;
    private DriveRunner driveRunner;
    private PlanRunner planRunner;

    @Override
    public void init() {
        NuggetHardware hardware = hardware();
        TankDrive tank = new TankDrive(hardware);
        localizer = new TankLocalizer(hardware, Trajectories.PARAMS, START);
        trajectories = new Trajectories(tank, localizer, hardware, Trajectories.PARAMS);
        driveRunner = new DriveRunner(hardware.dashboard, localizer::pose);
        planRunner = new PlanRunner();
        planRunner.run(lapOfTheField());
    }

    private Plan lapOfTheField() {
        return new Plan(
                leg(
                        "along the blue side",
                        path -> path.splineTo(new Vector2d(24, -44), 0).splineTo(new Vector2d(44, -24), Math.PI / 2)),
                leg(
                        "along the back wall",
                        path -> path.splineTo(new Vector2d(44, 24), Math.PI / 2)
                                .splineTo(new Vector2d(24, 44), Math.PI)),
                leg(
                        "along the red side",
                        path -> path.splineTo(new Vector2d(-24, 44), Math.PI)
                                .splineTo(new Vector2d(-44, 24), -Math.PI / 2)),
                leg(
                        "along the audience wall",
                        path -> path.splineTo(new Vector2d(-44, -24), -Math.PI / 2)
                                .splineTo(START.position, START.heading.toDouble())),
                leg("spin where it started", path -> path.turn(2 * Math.PI)));
    }

    private Step leg(String name, UnaryOperator<TrajectoryActionBuilder> path) {
        return new Step(
                name,
                () -> driveRunner.drive(
                        path.apply(trajectories.from(localizer.pose())).build()),
                driveRunner::done);
    }

    @Override
    public void loop() {
        localizer.loop();
        driveRunner.loop();
        planRunner.loop();
        telemetry.addData("Current Step:", planRunner.currentStep());
    }

    @Override
    public boolean done() {
        return planRunner.done();
    }

    @Override
    public String currentStep() {
        return planRunner.currentStep();
    }
}
