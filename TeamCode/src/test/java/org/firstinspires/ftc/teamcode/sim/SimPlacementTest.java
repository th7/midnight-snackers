package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.acmerobotics.roadrunner.Pose2d;
import java.io.File;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

/**
 * Placing a pose is geometry. The seam this holds is that it is <em>only</em> geometry: the
 * placement page, the run stream and the replay page all ask where the field lets a robot be, and
 * none of them is running a simulation.
 */
public class SimPlacementTest {
    private static final double DELTA = 0.001;
    private static final double HALF_ROBOT = SimPlacement.ROBOT_SIZE_IN / 2;

    /**
     * The gate. "The physics is not needed to place a pose" is the reason this module exists, and
     * a sentence in a javadoc is not a mechanism, so load it with the rigid-body engine and the
     * simulator itself forbidden and make it answer anyway. Before the split this failed: clamping
     * one pose loaded twelve dyn4j classes.
     */
    @Test
    public void placingAPoseNeedsNeitherTheRigidBodyEngineNorTheSimulatedRobot() throws Exception {
        List<String> forbidden = List.of("org.dyn4j.", SimRobot.class.getName());
        try (URLClassLoader loader = new URLClassLoader(classpath(), null) {
            @Override
            protected Class<?> loadClass(String name, boolean resolve) throws ClassNotFoundException {
                for (String banned : forbidden) {
                    if (name.startsWith(banned)) {
                        throw new ClassNotFoundException(
                                name + " must not be needed to place a pose; that is the seam this test holds");
                    }
                }
                return super.loadClass(name, resolve);
            }
        }) {
            Class<?> placement = Class.forName(SimPlacement.class.getName(), true, loader);
            Class<?> pose = Class.forName(Pose2d.class.getName(), true, loader);

            Object placed = placement
                    .getMethod("onTheField", pose)
                    .invoke(
                            null,
                            pose.getConstructor(double.class, double.class, double.class)
                                    .newInstance(1000.0, -1000.0, 0.0));

            assertNotNull("it answered, with no engine on the classpath it could reach", placed);
            assertEquals(
                    "and answered the same as it does here",
                    SimPlacement.onTheField(new Pose2d(1000, -1000, 0)).position.x,
                    (double) pose.getField("position")
                            .get(placed)
                            .getClass()
                            .getField("x")
                            .get(pose.getField("position").get(placed)),
                    DELTA);
        }
    }

    /** The same loader must still refuse the simulator, or the test above proves nothing. */
    @Test
    public void theGateWouldNoticeIfTheSimulatorCameBackInThroughTheBackDoor() throws Exception {
        try (URLClassLoader loader = new URLClassLoader(classpath(), null) {
            @Override
            protected Class<?> loadClass(String name, boolean resolve) throws ClassNotFoundException {
                if (name.startsWith("org.dyn4j.")) {
                    throw new ClassNotFoundException(name);
                }
                return super.loadClass(name, resolve);
            }
        }) {
            Class.forName(SimRobot.class.getName(), true, loader);
            fail("the simulated robot should not initialise without the rigid-body engine");
        } catch (ExceptionInInitializerError | NoClassDefFoundError | ClassNotFoundException expected) {
            // exactly the point: SimRobot does need dyn4j, and SimPlacement does not
        }
    }

    @Test
    public void aPoseBeyondAWallComesBackAgainstIt() {
        Pose2d placed = SimPlacement.onTheField(new Pose2d(1000, 0, 0));

        assertEquals(SimPlacement.FIELD_SIZE_IN / 2 - HALF_ROBOT, placed.position.x, DELTA);
        assertEquals(0, placed.position.y, DELTA);
        assertEquals("at the heading it was given", 0, placed.heading.toDouble(), DELTA);
    }

    @Test
    public void aPoseTheFieldAllowsIsTheSameObjectBack() {
        Pose2d open = new Pose2d(-60, 0, 0);

        assertSame(open, SimPlacement.onTheField(open));
    }

    /** Turned, the square reaches further, so the wall stops it sooner. */
    @Test
    public void aRobotAtFortyFiveDegreesIsStoppedFurtherFromTheWall() {
        double square = SimPlacement.onTheField(new Pose2d(1000, 0, 0)).position.x;
        double turned = SimPlacement.onTheField(new Pose2d(1000, 0, Math.PI / 4)).position.x;

        assertTrue("a diagonal robot reaches further", turned < square);
        assertEquals(SimPlacement.FIELD_SIZE_IN / 2 - HALF_ROBOT * Math.sqrt(2), turned, DELTA);
    }

    @Test
    public void aPoseInsideAnObstacleComesBackOutOfIt() {
        SimField.Obstacle obstacle = SimPlacement.FIELD.obstacles.get(0);
        double[] first = obstacle.footprint[0];

        Pose2d placed = SimPlacement.onTheField(new Pose2d(first[0], first[1], 0));

        assertTrue(
                "pushed clear of the obstacle it was dropped into", SimPlacement.clearOfTheObstacles(placed) == placed);
    }

    private static URL[] classpath() throws Exception {
        List<URL> urls = new ArrayList<>();
        for (String entry : System.getProperty("java.class.path").split(File.pathSeparator)) {
            urls.add(Paths.get(entry).toUri().toURL());
        }
        return urls.toArray(new URL[0]);
    }
}
