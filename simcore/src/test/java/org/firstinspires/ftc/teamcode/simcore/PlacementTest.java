package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class PlacementTest {
    private static final double DELTA = 1e-9;
    private static final double FIELD = 141;
    private static final double ROBOT = 18;
    private static final double LIMIT = FIELD / 2 - ROBOT / 2;

    private static final ConvexPolygon POST = square(20, 0, 5);

    private final Placement open = new Placement(Valid.length(FIELD), Valid.length(ROBOT), List.of());
    private final Placement withAPost = new Placement(Valid.length(FIELD), Valid.length(ROBOT), List.of(POST));

    private static ConvexPolygon square(double x, double y, double half) {
        return Valid.polygon(List.of(
                new Vec2(x - half, y - half),
                new Vec2(x + half, y - half),
                new Vec2(x + half, y + half),
                new Vec2(x - half, y + half)));
    }

    private static Pose pose(double x, double y, double radians) {
        return Valid.pose(x, y, radians);
    }

    private static Vec2 placed(Placed placed, Pose candidate) {
        return placed.from(candidate.position());
    }

    @Test
    public void aPoseTheFieldAllowsIsLeftAsGiven() {
        assertEquals(new Placed.AsGiven(), open.onTheField(pose(-40, 10, 1)));
        assertEquals(new Placed.AsGiven(), withAPost.onTheField(pose(-40, 10, 1)));
    }

    @Test
    public void aPoseBeyondAWallIsMovedAgainstItAtTheSameHeading() {
        Pose candidate = pose(1000, -30, 0);

        assertEquals(new Placed.Moved(new Vec2(LIMIT, -30)), open.onTheField(candidate));
    }

    @Test
    public void aPoseBeyondTwoWallsIsMovedIntoTheCorner() {
        assertEquals(new Placed.Moved(new Vec2(-LIMIT, LIMIT)), open.onTheField(pose(-1000, 1000, 0)));
    }

    @Test
    public void aTurnedRobotReachesFurtherSoStopsFurtherFromTheWall() {
        Vec2 turned = placed(open.onTheField(pose(1000, 0, Math.PI / 4)), pose(1000, 0, Math.PI / 4));

        assertEquals(FIELD / 2 - ROBOT / 2 * Math.sqrt(2), turned.x(), DELTA);
    }

    @Test
    public void aPoseInsideAnObstacleIsPushedTheShortWayOutOfIt() {
        Pose candidate = pose(24, 1, 0);

        Vec2 out = placed(withAPost.onTheField(candidate), candidate);

        assertEquals("out past the post's +x side by half the robot", 25 + ROBOT / 2, out.x(), DELTA);
        assertEquals(1, out.y(), DELTA);
        assertEquals(new Placed.AsGiven(), withAPost.clearOfTheObstacles(pose(out.x(), out.y(), 0)));
    }

    @Test
    public void aTurnedRobotIsPushedClearOfAnObstacle() {
        Pose candidate = pose(20, 12, Math.PI / 6);

        Vec2 out = placed(withAPost.onTheField(candidate), candidate);

        assertTrue(out.y() > 12);
        assertEquals(new Placed.AsGiven(), withAPost.clearOfTheObstacles(pose(out.x(), out.y(), Math.PI / 6)));
    }

    @Test
    public void aRobotPushedOutOfAnObstacleAtTheWallStillEndsInsideTheField() {
        Placement atTheWall = new Placement(Valid.length(FIELD), Valid.length(ROBOT), List.of(square(55, 0, 5)));
        Pose candidate = pose(58, 0, 0);

        Vec2 out = placed(atTheWall.onTheField(candidate), candidate);

        assertEquals(LIMIT, out.x(), DELTA);
    }

    @Test
    public void clearingTheObstaclesLeavesAPoseInTheOpenAsGiven() {
        assertEquals(new Placed.AsGiven(), withAPost.clearOfTheObstacles(pose(1000, 1000, 0)));
    }
}
