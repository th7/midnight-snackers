package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// this class is missing when the robot runs
// maybe an explicit import helps get it out there somehow?
//import edu.wpi.first.math.proto.Controller;
public class ExampleSubsystem extends SubsystemBase {
    public final ElapsedTime runtime;
    public final Telemetry telemetry;
//    public Controller.ProtobufArmFeedforward derp;

    public ExampleSubsystem(ElapsedTime runtime, Telemetry telemetry) {
        this.runtime = runtime;
        this.telemetry = telemetry;
//        this.derp = null;
    }

    public Command outputTime() {
        AtomicReference<Double> stopOutputAt = new AtomicReference<>((double) 0);
        return func(
                () -> { stopOutputAt.set(runtime.seconds() + 2); },
                () -> { telemetry.addData("ExampleSubsystem.outputTime", stopOutputAt.get() - runtime.seconds()); },
                () -> runtime.seconds() < stopOutputAt.get(),
                noAfter()
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private FunctionalCommand func(Runnable start, Runnable periodic, BooleanSupplier finished, Consumer<Boolean> after) {
        return new FunctionalCommand(
                start,
                periodic,
                after,
                finished,
                this
        );
    }

    private Consumer<Boolean> noAfter() {
        return (unused) -> {};
    }
}