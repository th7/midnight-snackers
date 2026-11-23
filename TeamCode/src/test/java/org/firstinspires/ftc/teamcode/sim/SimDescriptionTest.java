package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Configuration;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.Constants;
import org.firstinspires.ftc.teamcode.simcore.Directions;
import org.firstinspires.ftc.teamcode.simcore.EncoderReads;
import org.firstinspires.ftc.teamcode.simcore.Facing;
import org.firstinspires.ftc.teamcode.simcore.HubMounting;
import org.firstinspires.ftc.teamcode.simcore.Mechanism;
import org.firstinspires.ftc.teamcode.simcore.MotorType;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimDescriptionTest {
    private static final Path COMMITTED = Paths.get(SimDescription.FILE);

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static void assertRefused(String saying, Checked<RobotDescription> read) {
        String said = read.fold(value -> "accepted " + value, rule -> rule);
        assertTrue(said, read instanceof Checked.Rejected && said.contains(saying));
    }

    private String committed() throws IOException {
        return new String(Files.readAllBytes(COMMITTED), StandardCharsets.UTF_8);
    }

    private Path written(String text) throws IOException {
        Path file = folder.getRoot().toPath().resolve(SimDescription.FILE);
        Files.write(file, text.getBytes(StandardCharsets.UTF_8));
        return file;
    }

    @Test
    public void theCommittedFileDescribesReginaldAsBuilt() {
        assertEquals(RobotDescription.reginalds(), Valid.value(SimDescription.read(COMMITTED)));
    }

    @Test
    public void aValueOutOfItsRangeIsRefusedNamingTheFileTheValueAndItsRange() throws IOException {
        Path file = written(committed().replace("\"mass_kg\": 15", "\"mass_kg\": 40"));

        Checked<RobotDescription> read = SimDescription.read(file);

        assertRefused(file.toString(), read);
        assertRefused("Mass is from 1.0 to 30.0 kg, not 40.0", read);
    }

    @Test
    public void anEditedValueIsReadAsItStands() throws IOException {
        Path file = written(committed()
                .replace("\"mass_kg\": 15", "\"mass_kg\": 18")
                .replace("{\"logo\": \"RIGHT\", \"usb\": \"UP\"}", "{\"logo\": \"DOWN\", \"usb\": \"RIGHT\"}"));

        RobotDescription read = Valid.value(SimDescription.read(file));

        assertEquals(18, read.constants().value(Constant.MASS_KG), 0);
        assertEquals(Valid.value(HubMounting.of(Facing.DOWN, Facing.RIGHT)), read.hub());
    }

    @Test
    public void aReversedMechanismIsReadAsItStands() throws IOException {
        Path file = written(committed().replace("\"intake\": \"takes_in\"", "\"intake\": \"pushes_out\""));

        RobotDescription read = Valid.value(SimDescription.read(file));

        assertTrue(read.directions().reversed(Mechanism.INTAKE));
    }

    @Test
    public void theDirectionsNameEveryMechanismAndOnlyTheWaysEachHas() throws IOException {
        assertRefused(
                "directions is missing",
                SimDescription.read(written(committed()
                        .replace(
                                "\"directions\": {\n    \"intake\": \"takes_in\", \"spinner\": \"turns_clockwise\",\n"
                                        + "    \"flywheel_front\": \"turns_counterclockwise\","
                                        + " \"flywheel_back\": \"turns_counterclockwise\"\n  },",
                                ""))));
        assertRefused(
                "the robot description's directions name every mechanism, and intake is missing",
                SimDescription.read(written(committed().replace("\"intake\": \"takes_in\", ", ""))));
        assertRefused(
                "a mechanism is intake, spinner, flywheel_front, flywheel_back, not 'outtake'",
                SimDescription.read(
                        written(committed().replace("\"intake\": \"takes_in\"", "\"outtake\": \"takes_in\""))));
        assertRefused(
                "the intake takes_in or pushes_out on positive power, not 'spins'",
                SimDescription.read(written(committed().replace("\"intake\": \"takes_in\"", "\"intake\": \"spins\""))));
    }

    @Test
    public void theCommittedFileNamesEachDeviceOnlyByTheNameItIsCommittedUnder() throws IOException {
        assertRefused(
                "a configured device names only name, kind, type, encoder, not 'committed'",
                SimDescription.read(written(committed()
                        .replace(
                                "{\"name\": \"intake\", \"kind\": \"motor\",",
                                "{\"name\": \"intakeMotor\", \"committed\": \"intake\", \"kind\": \"motor\","))));
    }

    @Test
    public void aFileIsReadWholeAndRefusedForAnythingMissingUnknownOrMisnamed() throws IOException {
        assertRefused("mass_kg", SimDescription.read(written(committed().replace("\"mass_kg\": 15,", ""))));
        assertRefused(
                "'gravity'",
                SimDescription.read(
                        written(committed().replace("\"mass_kg\": 15,", "\"mass_kg\": 15, \"gravity\": 9.8,"))));
        assertRefused(
                "hub is missing",
                SimDescription.read(
                        written(committed().replace("\"hub\": {\"logo\": \"RIGHT\", \"usb\": \"UP\"},", ""))));
        assertRefused(
                "'SIDEWAYS'",
                SimDescription.read(written(committed().replace("\"logo\": \"RIGHT\"", "\"logo\": \"SIDEWAYS\""))));
        assertRefused(
                "the hub's logo and its USB ports face along different axes",
                SimDescription.read(written(committed().replace("\"logo\": \"RIGHT\"", "\"logo\": \"UP\""))));
        assertRefused(
                "'NeveRest 40'",
                SimDescription.read(
                        written(committed().replaceFirst("REV Robotics UltraPlanetary HD Hex Motor", "NeveRest 40"))));
        assertRefused(
                "'the moon'",
                SimDescription.read(
                        written(committed().replace("\"encoder\": \"nothing\"", "\"encoder\": \"the moon\""))));
        assertRefused(
                "'periscope'",
                SimDescription.read(written(committed().replace("\"kind\": \"imu\"", "\"kind\": \"periscope\""))));
        assertRefused(
                "two devices are named 'intake'",
                SimDescription.read(written(committed().replace("\"flywheelBack\"", "\"intake\""))));
        assertRefused("not JSON", SimDescription.read(written("{\"configuration\": [")));
        assertRefused("not JSON", SimDescription.read(written(committed() + "{}")));
        assertRefused("not JSON", SimDescription.read(written("// Reginald\n" + committed())));
        assertRefused(
                "could not read", SimDescription.read(folder.getRoot().toPath().resolve("absent.json")));
    }

    @Test
    public void theStartLineCarriesOnlyWhatDiffersFromReginaldAsBuiltAndIsReadBackAsTheSameDescription() {
        RobotDescription differs = RobotDescription.of(
                Valid.value(Configuration.of(List.of(
                        new Configuration.Motor(
                                "rightBack", MotorType.ULTRAPLANETARY_HD_HEX, EncoderReads.PARALLEL_DEAD_WHEEL),
                        new Configuration.Motor(
                                "leftFront", MotorType.ULTRAPLANETARY_HD_HEX, EncoderReads.PERPENDICULAR_DEAD_WHEEL),
                        new Configuration.Imu("imu")))),
                Valid.value(HubMounting.of(Facing.DOWN, Facing.RIGHT)),
                Valid.value(Constants.of(Map.of(Constant.MASS_KG, 18.0))),
                Directions.reginalds().with(Mechanism.INTAKE, true));
        JsonObject line = new JsonObject();
        JsonObject asBuilt = new JsonObject();

        SimDescription.addWhatDiffers(differs, line);
        SimDescription.addWhatDiffers(RobotDescription.reginalds(), asBuilt);
        JsonObject heard = new Gson().fromJson(line.toString(), JsonObject.class);

        assertEquals(differs, Valid.value(SimDescription.fromStartLine(heard)));
        assertEquals(18, heard.getAsJsonObject("constants").get("mass_kg").getAsDouble(), 0);
        assertEquals(1, heard.getAsJsonObject("constants").size());
        assertEquals(
                "pushes_out", heard.getAsJsonObject("directions").get("intake").getAsString());
        assertEquals("{}", asBuilt.toString());
        assertEquals(RobotDescription.reginalds(), Valid.value(SimDescription.fromStartLine(asBuilt)));
    }

    @Test
    public void aDescriptionIsDescribedAsBuiltOrByWhatDiffersFromTheBuiltDefault() {
        RobotDescription differs = RobotDescription.of(
                Configuration.reginalds(),
                Valid.value(HubMounting.of(Facing.DOWN, Facing.RIGHT)),
                Valid.value(Constants.of(Map.of(Constant.MASS_KG, 18.0))),
                Directions.reginalds().with(Mechanism.INTAKE, true));

        String said = SimDescription.described(differs);

        assertEquals("as built", SimDescription.described(RobotDescription.reginalds()));
        assertTrue(said, said.contains("Mass 18.0 kg (built 15.0)"));
        assertTrue(said, said.contains("hub logo DOWN, USB RIGHT (built logo RIGHT, USB UP)"));
        assertTrue(said, said.contains("intake pushes out on positive power (built takes in)"));
        assertFalse(said, said.contains("configuration"));
    }
}
