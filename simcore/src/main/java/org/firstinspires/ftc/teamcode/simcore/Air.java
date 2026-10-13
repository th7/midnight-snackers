package org.firstinspires.ftc.teamcode.simcore;

public final class Air {
    private static final double ABSOLUTE_ZERO_CELSIUS = -273.15;
    private static final double TOP_OF_THE_ATMOSPHERE_METRES = 44330;
    private static final double SEA_LEVEL_PASCALS = 101325;
    private static final double SEA_LEVEL_KELVIN = 288.15;
    private static final double LAPSE_KELVIN_PER_METRE = 0.0065;
    private static final double PRESSURE_EXPONENT = 5.25588;
    private static final double TETENS_PASCALS = 610.78;
    private static final double TETENS_SCALE = 17.27;
    private static final double TETENS_OFFSET_CELSIUS = 237.3;
    private static final double DRY_AIR_JOULES_PER_KILOGRAM_KELVIN = 287.058;
    private static final double WATER_VAPOUR_JOULES_PER_KILOGRAM_KELVIN = 461.495;
    private static final double PERCENT = 100;

    public record Motion(Vec3 acceleration, Vec3 spinRate) {}

    private final double density;
    private final Vec3 wind;
    private final double drag;
    private final double dragPerSpin;
    private final double liftSlope;
    private final double liftLimit;
    private final double spinDown;
    private final double addedMass;

    private Air(
            double density,
            Vec3 wind,
            double drag,
            double dragPerSpin,
            double liftSlope,
            double liftLimit,
            double spinDown,
            double addedMass) {
        this.density = density;
        this.wind = wind;
        this.drag = drag;
        this.dragPerSpin = dragPerSpin;
        this.liftSlope = liftSlope;
        this.liftLimit = liftLimit;
        this.spinDown = spinDown;
        this.addedMass = addedMass;
    }

    public static Air vacuum() {
        return new Air(0, Vec3.zero(), 0, 0, 0, 1, 0, 0);
    }

    public static Checked<Double> densityOf(double celsius, double elevationMetres, double humidityPercent) {
        if (!(celsius > ABSOLUTE_ZERO_CELSIUS && Double.isFinite(celsius))) {
            return Checked.rejected("a temperature is a finite number of degrees Celsius above absolute zero, "
                    + ABSOLUTE_ZERO_CELSIUS + ", not " + celsius);
        }
        if (!(elevationMetres < TOP_OF_THE_ATMOSPHERE_METRES && Double.isFinite(elevationMetres))) {
            return Checked.rejected("an elevation is a finite number of metres below " + TOP_OF_THE_ATMOSPHERE_METRES
                    + ", where the standard atmosphere's pressure runs out, not " + elevationMetres);
        }
        if (!(humidityPercent >= 0 && humidityPercent <= PERCENT)) {
            return Checked.rejected("a relative humidity is a percentage from 0 to 100, not " + humidityPercent);
        }
        double pascals = SEA_LEVEL_PASCALS
                * StrictMath.pow(1 - LAPSE_KELVIN_PER_METRE * elevationMetres / SEA_LEVEL_KELVIN, PRESSURE_EXPONENT);
        double kelvin = celsius - ABSOLUTE_ZERO_CELSIUS;
        double vapour = StrictMath.min(humidityPercent / PERCENT * saturatedPascals(celsius), pascals);
        double kilogramsPerCubicMetre = (pascals - vapour) / (DRY_AIR_JOULES_PER_KILOGRAM_KELVIN * kelvin)
                + vapour / (WATER_VAPOUR_JOULES_PER_KILOGRAM_KELVIN * kelvin);
        if (!(kilogramsPerCubicMetre > 0 && Double.isFinite(kilogramsPerCubicMetre))) {
            return Checked.rejected("air at " + celsius + " C, " + elevationMetres + " m and " + humidityPercent
                    + " % humidity has no finite, positive density");
        }
        return Checked.ok(kilogramsPerCubicMetre);
    }

    private static double saturatedPascals(double celsius) {
        double aboveTheFormulasFloor = celsius + TETENS_OFFSET_CELSIUS;
        return aboveTheFormulasFloor > 0
                ? TETENS_PASCALS * StrictMath.exp(TETENS_SCALE * celsius / aboveTheFormulasFloor)
                : 0;
    }

    public static Checked<Air> of(
            double kilogramsPerCubicMetre,
            Vec3 windInPerS,
            double drag,
            double dragPerSpin,
            double liftSlope,
            double liftLimit,
            double spinDown,
            double addedMass) {
        if (!(kilogramsPerCubicMetre > 0 && Double.isFinite(kilogramsPerCubicMetre))) {
            return Checked.rejected("air's density is a positive, finite number of kilograms per cubic metre, not "
                    + kilogramsPerCubicMetre);
        }
        if (!windInPerS.isFinite()) {
            return Checked.rejected("a wind is a finite velocity in inches per second, not " + windInPerS);
        }
        if (!isFiniteAndNotNegative(drag)) {
            return Checked.rejected("a drag coefficient is a finite number at least 0, not " + drag);
        }
        if (!isFiniteAndNotNegative(dragPerSpin)) {
            return Checked.rejected(
                    "the drag each unit of spin parameter adds is a finite number at least 0, not " + dragPerSpin);
        }
        if (!isFiniteAndNotNegative(liftSlope)) {
            return Checked.rejected("a lift slope, how fast lift first rises with the spin parameter, is a finite"
                    + " number at least 0, not " + liftSlope);
        }
        if (!(liftLimit > 0 && Double.isFinite(liftLimit))) {
            return Checked.rejected(
                    "a lift limit, the most lift spin can give, is a positive, finite number, not " + liftLimit);
        }
        if (!isFiniteAndNotNegative(spinDown)) {
            return Checked.rejected("a spin-down coefficient is a finite number at least 0, not " + spinDown);
        }
        if (!isFiniteAndNotNegative(addedMass)) {
            return Checked.rejected("an added-mass coefficient, the share of the air it displaces that a moving ball"
                    + " carries along, is a finite number at least 0, not " + addedMass);
        }
        return Checked.ok(new Air(
                kilogramsPerCubicMetre, windInPerS, drag, dragPerSpin, liftSlope, liftLimit, spinDown, addedMass));
    }

    private static boolean isFiniteAndNotNegative(double value) {
        return value >= 0 && Double.isFinite(value);
    }

    public double density() {
        return density;
    }

    public Motion motion(Ball ball, Vec3 velocity, Vec3 spin) {
        Vec3 through = velocity.minus(wind);
        double squaredSpeed = through.dot(through);
        double speed = StrictMath.sqrt(squaredSpeed);
        double displaced = density * ball.volumeCubicMetres();
        double moving = ball.kilograms() + addedMass * displaced;
        Vec3 falling = new Vec3(0, 0, -Flight.GRAVITY_IN_PER_S2 * ((ball.kilograms() - displaced) / moving));
        if (!(speed > 0)) {
            return new Motion(falling, Vec3.zero());
        }
        double perInch = density * ball.areaSquareMetres() / (2 * moving) * Length.METRES_PER_INCH;
        double radius = ball.radius();
        Vec3 sideways = spin.cross(through);
        double sidewaysSize = sizeOf(sideways);
        Vec3 dragging = through.times(-perInch * (drag * speed + dragPerSpin * radius * (sidewaysSize / speed)));
        double liftDenominator = liftLimit * squaredSpeed + liftSlope * radius * sidewaysSize;
        Vec3 lifting = liftDenominator > 0
                ? sideways.times(perInch * liftSlope * radius * liftLimit * squaredSpeed / liftDenominator)
                : Vec3.zero();
        double spinsDownPerInch = spinDown
                * density
                * ball.areaSquareMetres()
                / (ball.inertiaFactor() * ball.kilograms())
                * Length.METRES_PER_INCH;
        return new Motion(dragging.plus(lifting).plus(falling), spin.times(-spinsDownPerInch * speed));
    }

    public double spinParameter(Ball ball, Vec3 velocity, Vec3 spin) {
        Vec3 through = velocity.minus(wind);
        double speed = sizeOf(through);
        if (!(speed > 0)) {
            return 0;
        }
        return ball.radius() * (sizeOf(spin.cross(through)) / speed) / speed;
    }

    public double dragCoefficient(double s) {
        return drag + dragPerSpin * StrictMath.abs(s);
    }

    public double liftCoefficient(double s) {
        return liftSlope * s / (1 + liftSlope * StrictMath.abs(s) / liftLimit);
    }

    private static double sizeOf(Vec3 v) {
        return StrictMath.sqrt(v.dot(v));
    }
}
