package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public final class FieldOffer {
    public static final int VERSION = 1;

    private static final int SHA256_HEX_DIGITS = 64;

    public record Said(String resolution, String file, long bytes, String sha256) {}

    public static final class Offered {
        private final FieldModel model;
        private final long bytes;
        private final String sha256;

        private Offered(FieldModel model, long bytes, String sha256) {
            this.model = model;
            this.bytes = bytes;
            this.sha256 = sha256;
        }

        static Checked<Offered> of(Said said) {
            return FieldModel.named(said.resolution())
                    .then(model -> model.file().equals(said.file())
                            ? Checked.ok(model)
                            : Checked.<FieldModel>rejected("the " + model.asked() + " field model is " + model.file()
                                    + ", not " + said.file()))
                    .then(model -> said.bytes() >= 0
                            ? Checked.ok(model)
                            : Checked.<FieldModel>rejected(model.file() + " cannot be " + said.bytes() + " bytes long"))
                    .then(model -> isSha256(said.sha256())
                            ? Checked.ok(new Offered(model, said.bytes(), said.sha256()))
                            : Checked.<Offered>rejected("the digest of " + model.file()
                                    + " is a SHA-256 in 64 lowercase hex digits, not " + said.sha256()));
        }

        public FieldModel model() {
            return model;
        }

        public long bytes() {
            return bytes;
        }

        public String sha256() {
            return sha256;
        }
    }

    private final List<Offered> models;

    private FieldOffer(List<Offered> models) {
        this.models = Collections.unmodifiableList(new ArrayList<>(models));
    }

    public static Checked<FieldOffer> read(int version, List<Said> said) {
        if (version != VERSION) {
            return Checked.rejected("the host offers its field models in version " + version
                    + " and this simulator reads version " + VERSION
                    + (version > VERSION ? ": pull develop for a simulator that reads it" : ""));
        }
        Checked<List<Offered>> read = Checked.ok(List.of());
        for (Said one : said) {
            read = read.then(sofar -> Offered.of(one).then(offered -> added(sofar, offered)));
        }
        return read.map(FieldOffer::new);
    }

    private static Checked<List<Offered>> added(List<Offered> sofar, Offered offered) {
        for (Offered already : sofar) {
            if (already.model == offered.model) {
                return Checked.rejected("the host offers the " + offered.model.asked() + " field model twice");
            }
        }
        List<Offered> more = new ArrayList<>(sofar);
        more.add(offered);
        return Checked.ok(more);
    }

    public List<Offered> models() {
        return models;
    }

    public List<Offered> toFetch(Map<FieldModel, String> keptSha256) {
        List<Offered> fetch = new ArrayList<>();
        for (Offered one : models) {
            if (!one.sha256.equals(keptSha256.get(one.model))) {
                fetch.add(one);
            }
        }
        return Collections.unmodifiableList(fetch);
    }

    public Set<FieldModel> offered() {
        Set<FieldModel> out = EnumSet.noneOf(FieldModel.class);
        for (Offered one : models) {
            out.add(one.model);
        }
        return out;
    }

    private static boolean isSha256(String digest) {
        char[] digits = digest.toCharArray();
        if (digits.length != SHA256_HEX_DIGITS) {
            return false;
        }
        for (char c : digits) {
            if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {
                return false;
            }
        }
        return true;
    }
}
