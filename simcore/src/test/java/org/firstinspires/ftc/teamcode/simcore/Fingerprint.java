package org.firstinspires.ftc.teamcode.simcore;

import java.util.List;

record Fingerprint(
        long work,
        String frames,
        String impacts,
        String crossings,
        String apex,
        String firstBounce,
        String tracks,
        String finish) {

    static Fingerprint of(Shot shot) {
        Shot.Story story = shot.story();
        Fold frames = new Fold();
        for (Shot.Frame frame : story.frames()) {
            frames.frame(frame);
        }
        Fold impacts = new Fold();
        for (Shot.Impact impact : story.impacts()) {
            impacts.impact(impact);
        }
        Fold crossings = new Fold();
        for (Shot.Crossed crossed : story.crossings()) {
            crossings.crossed(crossed);
        }
        Fold firstBounce = new Fold().add(story.firstBounce().isPresent() ? 1 : 0);
        story.firstBounce()
                .ifPresent(
                        bounce -> firstBounce.add(bounce.t()).add(bounce.at()).add(bounce.range()));
        Fold tracks = new Fold();
        for (Shot.Track track : story.pieces()) {
            tracks.track(track);
        }
        return new Fingerprint(
                story.work(),
                frames.hex(),
                impacts.hex(),
                crossings.hex(),
                new Fold().frame(story.apex()).hex(),
                firstBounce.hex(),
                tracks.hex(),
                new Fold().finish(shot).add(story.simulated()).hex());
    }

    private static final class Fold {
        private static final long MIX = 0x9E3779B97F4A7C15L;
        private long hash = 0xcbf29ce484222325L;
        private long count;

        Fold add(long value) {
            hash = (hash ^ value) * MIX;
            hash ^= hash >>> 29;
            count++;
            return this;
        }

        Fold add(double value) {
            return add(Double.doubleToRawLongBits(value));
        }

        Fold add(boolean value) {
            return add(value ? 1 : 0);
        }

        Fold add(Seconds t) {
            return add(t.value());
        }

        Fold add(Vec3 v) {
            return add(v.x()).add(v.y()).add(v.z());
        }

        Fold add(Turn turn) {
            return add(turn.w()).add(turn.x()).add(turn.y()).add(turn.z());
        }

        Fold add(String text) {
            add(text.length());
            for (char c : text.toCharArray()) {
                add(c);
            }
            return this;
        }

        Fold add(Enum<?> value) {
            return add(value.name());
        }

        Fold frame(Shot.Frame frame) {
            Shot.Moving pollen = frame.pollen();
            return add(frame.t())
                    .add(pollen.at())
                    .add(pollen.velocity())
                    .add(pollen.spin())
                    .add(pollen.turn())
                    .add(pollen.phase());
        }

        Fold impact(Shot.Impact impact) {
            Contacts.Hit hit = impact.hit();
            return add(impact.t())
                    .add(impact.at())
                    .add(impact.normal())
                    .add(hit.ball())
                    .add(hit.against())
                    .add(hit.surface())
                    .add(hit.normal())
                    .add(hit.approach())
                    .add(hit.slip())
                    .add(hit.restitution())
                    .add(hit.gripped())
                    .add(hit.normalImpulse())
                    .add(impact.velocityBefore())
                    .add(impact.spinBefore())
                    .add(impact.velocityAfter())
                    .add(impact.spinAfter());
        }

        Fold crossed(Shot.Crossed crossed) {
            Colliders.Crossing crossing = crossed.crossing();
            return add(crossed.t())
                    .add(crossing.cell())
                    .add(crossing.alliance())
                    .add(crossing.upturned())
                    .add(crossing.into())
                    .add(crossing.fraction())
                    .add(crossing.at())
                    .add(crossing.offset())
                    .add(crossing.rim())
                    .add(crossing.angleDegrees())
                    .add(crossed.speed());
        }

        Fold track(Shot.Track track) {
            add(track.piece().name());
            List<Shot.Placed> samples = track.samples();
            for (Shot.Placed sample : samples) {
                add(sample.t()).add(sample.at()).add(sample.turn());
            }
            if (track.fate() instanceof Shot.Fate.Gone gone) {
                add("gone").add(gone.t()).add(gone.side());
            } else {
                add("on the field");
            }
            return this;
        }

        Fold finish(Shot shot) {
            if (!(shot instanceof Shot.Finished finished)) {
                return add("flying");
            }
            Shot.Finish finish = finished.finish();
            add(finish.kind()).add(finish.says()).add(finish.t());
            if (finish instanceof Shot.Finish.AtRest rest) {
                add(rest.at()).add(rest.where().kind()).add(rest.where().says());
            }
            if (finish instanceof Shot.Finish.LeftTheField left) {
                add(left.at()).add(left.side());
            }
            return this;
        }

        String hex() {
            return count + ":" + String.format("%016x", hash);
        }
    }
}
