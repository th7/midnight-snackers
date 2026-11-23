package org.firstinspires.ftc.reginald.tools;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.OptionalDouble;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@TeleOp(name = "Flywheel step", group = "Measure")
public class FlywheelStep extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    public static final double TARGET_TICKS_PER_SECOND = 1200;
    public static final double RECORD_SECONDS = 10;
    public static final double TICKS_PER_TURN = 28;
    public static final double SETTLED_FRACTION = 0.02;
    public static final String FRONT = "flywheelFront";
    public static final String BACK = "flywheelBack";
    public static final String HEADER =
            "seconds,front_ticks_per_second,back_ticks_per_second,front_amps,back_amps,battery_volts";

    private enum State {
        WAITING,
        RECORDING,
        DONE
    }

    private final LongSupplier nanoClock;
    private final Supplier<File> saveTo;
    private final StringBuilder rows = new StringBuilder();
    private final StepResponse frontResponse = new StepResponse(TARGET_TICKS_PER_SECOND);
    private final StepResponse backResponse = new StepResponse(TARGET_TICKS_PER_SECOND);
    private DcMotorEx front;
    private DcMotorEx back;
    private VoltageSensor battery;
    private PIDFCoefficients frontGains;
    private PIDFCoefficients backGains;
    private State state = State.WAITING;
    private long startedAt;
    private double voltsAtStart;
    private double lastSeconds;
    private double lastVolts;
    private int samples;
    private List<String> summary = List.of();

    public FlywheelStep() {
        this(System::nanoTime, () -> AppUtil.ROBOT_DATA_DIR);
    }

    public FlywheelStep(LongSupplier nanoClock, Supplier<File> saveTo) {
        this.nanoClock = nanoClock;
        this.saveTo = saveTo;
    }

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        front = hardwareMap.get(DcMotorEx.class, FRONT);
        back = hardwareMap.get(DcMotorEx.class, BACK);
        front.setDirection(DcMotorSimple.Direction.REVERSE);
        back.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx flywheel : List.of(front, back)) {
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        frontGains = front.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        backGains = back.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        battery = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void init_loop() {
        telemetry.addLine(String.format(
                Locale.US,
                "Start spins both flywheels from rest to %.0f ticks/s and records them for %.0f s.",
                TARGET_TICKS_PER_SECOND,
                RECORD_SECONDS));
        telemetry.addLine("Load no pollen, and let both wheels come to rest first.");
        telemetry.addData(FRONT + " PIDF", gains(frontGains));
        telemetry.addData(BACK + " PIDF", gains(backGains));
        telemetry.addData("battery volts", battery.getVoltage());
    }

    @Override
    public void start() {
        startedAt = nanoClock.getAsLong();
        voltsAtStart = battery.getVoltage();
        state = State.RECORDING;
    }

    @Override
    public void loop() {
        if (state == State.RECORDING) {
            double seconds = (nanoClock.getAsLong() - startedAt) / 1e9;
            if (seconds > RECORD_SECONDS) {
                finish();
            } else {
                record(seconds);
            }
        }
        if (state == State.DONE) {
            for (String line : summary) {
                telemetry.addLine(line);
            }
        }
    }

    @Override
    public void stop() {
        if (state == State.RECORDING) {
            finish();
        }
        front.setPower(0);
        back.setPower(0);
    }

    private void record(double seconds) {
        front.setVelocity(TARGET_TICKS_PER_SECOND);
        back.setVelocity(TARGET_TICKS_PER_SECOND);
        double frontVelocity = front.getVelocity();
        double backVelocity = back.getVelocity();
        double frontAmps = front.getCurrent(CurrentUnit.AMPS);
        double backAmps = back.getCurrent(CurrentUnit.AMPS);
        lastVolts = battery.getVoltage();
        lastSeconds = seconds;
        samples++;
        frontResponse.add(seconds, frontVelocity);
        backResponse.add(seconds, backVelocity);
        rows.append(String.format(
                        Locale.US,
                        "%.4f,%.1f,%.1f,%.3f,%.3f,%.3f",
                        seconds,
                        frontVelocity,
                        backVelocity,
                        frontAmps,
                        backAmps,
                        lastVolts))
                .append('\n');
        telemetry.addData("seconds", "%.2f", seconds);
        telemetry.addData(FRONT + " rpm", "%.0f", rpm(frontVelocity));
        telemetry.addData(BACK + " rpm", "%.0f", rpm(backVelocity));
    }

    private void finish() {
        front.setPower(0);
        back.setPower(0);
        state = State.DONE;
        List<String> lines = new ArrayList<>();
        lines.add("target ticks per second: " + TARGET_TICKS_PER_SECOND);
        lines.add(String.format(Locale.US, "target rpm: %.0f", rpm(TARGET_TICKS_PER_SECOND)));
        lines.add(FRONT + " RUN_USING_ENCODER PIDF: " + gains(frontGains));
        lines.add(BACK + " RUN_USING_ENCODER PIDF: " + gains(backGains));
        lines.add(String.format(Locale.US, "battery volts at start: %.3f", voltsAtStart));
        lines.add(String.format(Locale.US, "battery volts at end: %.3f", lastVolts));
        lines.add(String.format(Locale.US, "samples: %d over %.2f s", samples, lastSeconds));
        lines.addAll(described(FRONT, frontResponse));
        lines.addAll(described(BACK, backResponse));
        lines.addAll(saved(lines));
        summary = lines;
    }

    private List<String> described(String name, StepResponse response) {
        List<String> lines = new ArrayList<>();
        response.peak()
                .ifPresent(peak -> lines.add(String.format(
                        Locale.US,
                        "%s peak: %.0f ticks/s (%.0f rpm) at %.2f s",
                        name,
                        peak.value(),
                        rpm(peak.value()),
                        peak.seconds())));
        OptionalDouble settled = response.settledWithin(SETTLED_FRACTION);
        lines.add(
                settled.isPresent()
                        ? String.format(
                                Locale.US,
                                "%s settled within %.0f%% at %.2f s",
                                name,
                                SETTLED_FRACTION * 100,
                                settled.getAsDouble())
                        : String.format(
                                Locale.US,
                                "%s not settled within %.0f%% by %.2f s",
                                name,
                                SETTLED_FRACTION * 100,
                                lastSeconds));
        return lines;
    }

    private List<String> saved(List<String> lines) {
        File folder = saveTo.get();
        String stem = "flywheel-step-" + new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        File csv = new File(folder, stem + ".csv");
        File txt = new File(folder, stem + ".txt");
        try {
            if (!folder.isDirectory() && !folder.mkdirs()) {
                throw new IOException("could not make " + folder);
            }
            try (Writer writer = new FileWriter(csv)) {
                writer.write(HEADER + "\n" + rows);
            }
            try (Writer writer = new FileWriter(txt)) {
                writer.write(String.join("\n", lines) + "\n");
            }
            return List.of("saved: " + csv.getPath(), "saved: " + txt.getPath());
        } catch (IOException e) {
            return List.of("could not save the recording: " + e.getMessage());
        }
    }

    private static double rpm(double ticksPerSecond) {
        return ticksPerSecond / TICKS_PER_TURN * 60;
    }

    private static String gains(PIDFCoefficients gains) {
        return "p=" + gains.p + " i=" + gains.i + " d=" + gains.d + " f=" + gains.f + " algorithm=" + gains.algorithm;
    }
}
