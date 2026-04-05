package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    public final ElapsedTime runtime;
    public final Telemetry telemetry;

    public ExampleSubsystem(ElapsedTime runtime, Telemetry telemetry) {
        this.runtime = runtime;
        this.telemetry = telemetry;
    }

//    public Command outputTime() {
//        AtomicReference<Double> stopOutputAt = new AtomicReference<>();
//        return command(
//                () -> stopOutputAt.set(runtime.seconds() + 2),
//                () -> telemetry.addData("ExampleSubsystem.outputTime", stopOutputAt.get() - runtime.seconds()),
//                () -> runtime.seconds() > stopOutputAt.get()
//        );
//    }

//    public Command outputTime() {
//        return command(
//            () -> {},
//            this::outputTimeNow
//        ).withTimeout(2);
//    }

    //    public Command outputTime() {
//        AtomicReference<Double> stopOutputAt = new AtomicReference<>();
//        return command(
//                () -> {},
//                this::outputTimeNow,
//                () -> false
//        );
//    }
    public Command outputTime() {
        return run(this::outputTimeNow).withTimeout(2);
    }

    public Command outputTwice() {
        return outputTime()
                .andThen(Commands.waitSeconds(2))
                .andThen(outputTime());
    }

    private void outputTimeNow() {
        telemetry.addData("ExampleSubsystem.outputTime", runtime.seconds());
    }

    @Override
    public void periodic() {
        // This method will be called once per loop
    }

    private FunctionalCommand command(Runnable start, Runnable periodic, BooleanSupplier finished, Consumer<Boolean> after) {
        return new FunctionalCommand(
                start,
                periodic,
                after,
                finished,
                this
        );
    }

    private FunctionalCommand command(Runnable start, Runnable periodic, BooleanSupplier finished) {
        return new FunctionalCommand(
                start,
                periodic,
                (unused) -> {},
                finished,
                this
        );
    }

    private FunctionalCommand command(Runnable start, Runnable periodic) {
        return new FunctionalCommand(
                start,
                periodic,
                (unused) -> {},
                () -> false,
                this
        );
    }
}