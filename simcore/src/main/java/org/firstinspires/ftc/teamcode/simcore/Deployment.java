package org.firstinspires.ftc.teamcode.simcore;

public final class Deployment {
    public enum Outcome {
        INSTALLED,
        FAILED,
        TIMED_OUT,
        INTERRUPTED,
        COULD_NOT_START,
        BUSY
    }

    private final Outcome outcome;
    private final String output;

    private Deployment(Outcome outcome, String output) {
        this.outcome = outcome;
        this.output = output;
    }

    public static Deployment exited(int exit, String output) {
        return new Deployment(exit == 0 ? Outcome.INSTALLED : Outcome.FAILED, output);
    }

    public static Deployment timedOut(String output) {
        return new Deployment(Outcome.TIMED_OUT, output);
    }

    public static Deployment interrupted(String output) {
        return new Deployment(Outcome.INTERRUPTED, output);
    }

    public static Deployment couldNotStart(String output) {
        return new Deployment(Outcome.COULD_NOT_START, output);
    }

    public static Deployment busy() {
        return new Deployment(Outcome.BUSY, "");
    }

    public Outcome outcome() {
        return outcome;
    }

    public String output() {
        return output;
    }

    public boolean ok() {
        return outcome == Outcome.INSTALLED;
    }

    public int status() {
        return switch (outcome) {
            case INSTALLED -> 200;
            case BUSY -> 409;
            case FAILED, TIMED_OUT, INTERRUPTED, COULD_NOT_START -> 500;
        };
    }

    public String message() {
        return switch (outcome) {
            case INSTALLED -> "deployed to the connected robot";
            case FAILED -> "deployment failed: check the build and ADB output";
            case TIMED_OUT -> "deployment timed out; the build was stopped";
            case INTERRUPTED -> "deployment interrupted; the build was stopped";
            case COULD_NOT_START -> "could not start deployment";
            case BUSY -> "another deployment is running; try again when it finishes";
        };
    }
}
