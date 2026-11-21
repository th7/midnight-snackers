package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class FeedFingerprintTest {
    private static final long MIX = 0x9E3779B97F4A7C15L;

    private static long fold(long hash, double value) {
        long mixed = (hash ^ Double.doubleToRawLongBits(value)) * MIX;
        return mixed ^ (mixed >>> 29);
    }

    private static long fold(long hash, Vec3 v) {
        return fold(fold(fold(hash, v.x()), v.y()), v.z());
    }

    private static String frames(Feed.Fed fed) {
        long hash = 0xcbf29ce484222325L;
        for (Feed.Frame frame : fed.frames()) {
            hash = fold(hash, frame.t().value());
            hash = fold(hash, frame.spinnerDegrees());
            hash = fold(hash, frame.spinnerRevolutionsPerSecond());
            hash = fold(hash, frame.spinnerLoadNewtonMetres());
            hash = fold(hash, frame.rampDegrees());
            hash = fold(hash, frame.rampTorqueNewtonMetres());
            for (Feed.Seen pollen : frame.pollen()) {
                hash = fold(fold(fold(hash, pollen.at()), pollen.velocity()), pollen.spin());
            }
        }
        return fed.frames().size() + ":" + String.format("%016x", hash);
    }

    private static String events(Feed.Fed fed) {
        long hash = 0xcbf29ce484222325L;
        for (Feed.Event event : fed.events()) {
            hash = fold(hash, event.at().value());
            if (event instanceof Feed.Event.Thrown thrown) {
                hash = fold(fold(fold(hash, thrown.from()), thrown.velocity()), thrown.spin());
            }
        }
        return fed.events().size() + ":" + String.format("%016x", hash);
    }

    private static Feed.Fed theDefaultFeed() {
        return Valid.value(Valid.value(Feed.Plan.defaults()).run());
    }

    @Test
    public void theDefaultFeedRunTwiceComesOutTheSameToTheBit() {
        assertEquals(theDefaultFeed(), theDefaultFeed());
    }

    @Test
    public void theDefaultFeedsWorkFramesEventsAndFinishArePinned() {
        Feed.Fed fed = theDefaultFeed();

        assertEquals(257930L, fed.work());
        assertEquals("248:c919260c16c2c0d3", frames(fed));
        assertEquals("2:032c7f0961f597b2", events(fed));
        assertEquals("AllLeft[at=0.9880000000000001 s]", fed.finish().toString());
    }
}
