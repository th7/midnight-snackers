package org.firstinspires.ftc.teamcode.simcore;

public final class RobotDescription {
    private static final double AN_INCH = 1;

    private final Configuration configuration;
    private final HubMounting hub;
    private final Constants constants;

    private RobotDescription(Configuration configuration, HubMounting hub, Constants constants) {
        this.configuration = configuration;
        this.hub = hub;
        this.constants = constants;
    }

    public static RobotDescription reginalds() {
        return new RobotDescription(Configuration.reginalds(), HubMounting.reginalds(), Constants.defaults());
    }

    public static RobotDescription of(Configuration configuration, HubMounting hub, Constants constants) {
        return new RobotDescription(configuration, hub, constants);
    }

    public RobotDescription with(Constants constants) {
        return new RobotDescription(configuration, hub, constants);
    }

    public Configuration configuration() {
        return configuration;
    }

    public HubMounting hub() {
        return hub;
    }

    public Constants constants() {
        return constants;
    }

    public Checked<DeadWheels> deadWheels() {
        return DeadWheels.at(
                constants.value(Constant.DEAD_WHEEL_IN_PER_TICK),
                constants.value(Constant.PARALLEL_DEAD_WHEEL_Y_IN),
                constants.value(Constant.PERPENDICULAR_DEAD_WHEEL_X_IN));
    }

    public Checked<Drivetrain> drivetrain(PerWheel<Noise.Motor> noise) {
        return Feedforward.of(
                        constants.value(Constant.DRIVE_KS_VOLTS),
                        constants.value(Constant.DRIVE_KV_VOLT_SECONDS_PER_IN),
                        constants.value(Constant.DRIVE_KA_VOLT_SECONDS_SQUARED_PER_IN))
                .then(perInch -> Drivetrain.of(perInch, AN_INCH, noise));
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof RobotDescription that
                && configuration.equals(that.configuration)
                && hub.equals(that.hub)
                && constants.equals(that.constants);
    }

    @Override
    public int hashCode() {
        return 31 * (31 * configuration.hashCode() + hub.hashCode()) + constants.hashCode();
    }
}
