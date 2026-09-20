package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

/**
 * What the suite spends on the few things that are dear: a child JVM, a compile, a git process, a
 * password hash, a project tree copied onto disk. Each is counted where it is spent -- inside the
 * one adapter that reaches it -- so the ledger is complete for the same reason nothing else may
 * reach any of them at all, which CostIsCountedWhereItIsSpentTest holds rather than a comment.
 *
 * <p>Counts are pinned in a budget and times are only ever printed: a count is the suite's and a
 * time is the machine's.
 */
public final class Cost {

    /** Where the suite writes its ledger, set by the build for the test JVM and by nothing else. */
    public static final String LEDGER = "suiteCost.ledger";

    /** Set by the build when the whole suite ran, so a filtered run says it could not judge. */
    public static final String WHOLE = "suiteCost.whole";

    /** Pin what the suite now spends, the way a golden trace is regenerated. */
    public static final String REGENERATE = "suiteCost.regenerate";

    public static final Path BUDGET = Paths.get(
            "src", "test", "resources", "org", "firstinspires", "ftc", "teamcode", "sim", "suite-budget.json");

    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    public enum Kind {
        CHILD_JVM("child-jvm", "a child JVM started"),
        COMPILE("compile", "a project's sources compiled"),
        GIT("git", "a git process run"),
        PASSWORD_HASH("password-hash", "a session secret put through scrypt"),
        FORMAT("format", "a Java file put through the formatter"),
        NODE("node", "a node process run"),
        PROJECT_COPY("project-copy", "a project tree copied onto disk");

        public final String key;
        public final String what;

        Kind(String key, String what) {
            this.key = key;
            this.what = what;
        }

        static Kind named(String key) {
            for (Kind kind : values()) {
                if (kind.key.equals(key)) {
                    return kind;
                }
            }
            throw new IllegalStateException(key
                    + " is not a kind of cost. A budget naming one that no longer exists is not read past in"
                    + " silence: take the line out of " + BUDGET + ", or put the kind back.");
        }
    }

    public static final class Tally {
        public final int count;
        public final double seconds;

        public Tally(int count, double seconds) {
            this.count = count;
            this.seconds = seconds;
        }
    }

    /** One spell of spending. Closing it is what records it, so try-with-resources cannot forget. */
    public interface Spent extends AutoCloseable {
        @Override
        void close();
    }

    /** What one JVM has spent. There is one of these per JVM; a test may make its own. */
    public static final class Ledger {
        private final Map<Kind, AtomicLong> counts = new EnumMap<>(Kind.class);
        private final Map<Kind, AtomicLong> nanos = new EnumMap<>(Kind.class);

        public Ledger() {
            for (Kind kind : Kind.values()) {
                counts.put(kind, new AtomicLong());
                nanos.put(kind, new AtomicLong());
            }
        }

        public Spent start(Kind kind) {
            long began = System.nanoTime();
            return new Spent() {
                private boolean recorded;

                @Override
                public void close() {
                    if (recorded) {
                        return;
                    }
                    recorded = true;
                    counts.get(kind).incrementAndGet();
                    nanos.get(kind).addAndGet(System.nanoTime() - began);
                }
            };
        }

        public Map<Kind, Tally> taken() {
            Map<Kind, Tally> taken = new EnumMap<>(Kind.class);
            for (Kind kind : Kind.values()) {
                taken.put(
                        kind,
                        new Tally((int) counts.get(kind).get(), nanos.get(kind).get() / 1e9));
            }
            return taken;
        }
    }

    private static final Ledger OF_THIS_JVM = new Ledger();

    static {
        String ledger = System.getProperty(LEDGER);
        if (ledger != null) {
            Runtime.getRuntime().addShutdownHook(new Thread(() -> writeUp(Paths.get(ledger)), "suite-cost"));
        }
    }

    private Cost() {}

    /** Count, and time, one spell of spending in this JVM's ledger. */
    public static Spent start(Kind kind) {
        return OF_THIS_JVM.start(kind);
    }

    public static Map<Kind, Tally> taken() {
        return OF_THIS_JVM.taken();
    }

    public static final class Verdict {
        public enum Outcome {
            MET,
            MISSED,
            COULD_NOT_JUDGE
        }

        public final Outcome outcome;
        public final String said;

        Verdict(Outcome outcome, String said) {
            this.outcome = outcome;
            this.said = said;
        }

        public boolean ok() {
            return outcome != Outcome.MISSED;
        }
    }

    public static Verdict verdictOn(Map<Kind, Tally> taken, Map<Kind, Integer> budget, boolean whole) {
        if (!whole) {
            return new Verdict(
                    Verdict.Outcome.COULD_NOT_JUDGE,
                    "this was part of the suite, or the suite split across JVMs, so what it cost could not"
                            + " judge the budget in " + BUDGET + ": a ledger is one JVM's, and only the whole"
                            + " suite in one JVM spends the whole of what is pinned. Run"
                            + " ./gradlew :TeamCode:testDebugUnitTest to check it.");
        }
        if (budget == null) {
            return new Verdict(
                    Verdict.Outcome.MISSED,
                    "there is no budget at " + BUDGET.toAbsolutePath() + ", so nothing is being checked."
                            + " Pin what the suite spends with " + how());
        }
        List<String> missed = new ArrayList<>();
        for (Kind kind : Kind.values()) {
            Integer pinned = budget.get(kind);
            int spent = taken.get(kind).count;
            if (pinned == null) {
                missed.add(String.format(
                        "nothing in the budget pins %s (%s), and the suite spent %d.", kind.key, kind.what, spent));
            } else if (spent > pinned) {
                missed.add(String.format(
                        "%s (%s): the budget pins %d, the suite spent %d -- %d more.",
                        kind.key, kind.what, pinned, spent, spent - pinned));
            } else if (spent < pinned) {
                missed.add(String.format(
                        "%s (%s): the budget pins %d, the suite spent %d -- %d fewer.",
                        kind.key, kind.what, pinned, spent, pinned - spent));
            }
        }
        if (missed.isEmpty()) {
            return new Verdict(Verdict.Outcome.MET, "the suite cost what " + BUDGET + " pins.");
        }
        return new Verdict(
                Verdict.Outcome.MISSED,
                "the suite no longer costs what " + BUDGET + " pins:\n  " + String.join("\n  ", missed)
                        + "\nA budget is a change detector, not a ceiling. If the change is one you meant, read it"
                        + " and then pin it with " + how());
    }

    private static String how() {
        return "./gradlew :TeamCode:testDebugUnitTest --rerun -D" + REGENERATE + "=true";
    }

    public static String report(Map<Kind, Tally> taken) {
        StringBuilder out = new StringBuilder("what the suite cost\n");
        for (Kind kind : Kind.values()) {
            Tally tally = taken.get(kind);
            out.append(String.format(
                    Locale.ROOT, "  %-14s %5d  %7.1fs   %s%n", kind.key, tally.count, tally.seconds, kind.what));
        }
        out.append("  (the count is pinned; the time is what was spent inside the call that does it,\n");
        out.append("   so a child JVM's own loading is in its run's time and not in the line above)\n");
        return out.toString();
    }

    public static String budgetJson(Map<Kind, Tally> taken) {
        JsonObject budget = new JsonObject();
        for (Kind kind : Kind.values()) {
            budget.addProperty(kind.key, taken.get(kind).count);
        }
        return GSON.toJson(budget) + "\n";
    }

    public static Map<Kind, Integer> budgetFrom(String json) {
        JsonObject read = GSON.fromJson(json, JsonObject.class);
        Map<Kind, Integer> budget = new EnumMap<>(Kind.class);
        for (Map.Entry<String, com.google.gson.JsonElement> entry : read.entrySet()) {
            budget.put(Kind.named(entry.getKey()), entry.getValue().getAsInt());
        }
        return budget;
    }

    /**
     * The whole of what the test JVM says about its spending: the ledger it took, the report a
     * person reads, and the verdict the build fails on. Written once, as the JVM goes.
     */
    static void writeUp(Path into) {
        Store store = new OnDiskStore();
        Map<Kind, Tally> taken = taken();
        boolean whole = Boolean.parseBoolean(System.getProperty(WHOLE, "false"));
        store.writeWhole(into.resolve("ledger.json"), budgetJson(taken).getBytes(StandardCharsets.UTF_8));
        store.writeWhole(into.resolve("report.txt"), report(taken).getBytes(StandardCharsets.UTF_8));
        if (whole && Boolean.getBoolean(REGENERATE)) {
            store.writeWhole(BUDGET, budgetJson(taken).getBytes(StandardCharsets.UTF_8));
            write(
                    store,
                    into,
                    Verdict.Outcome.MISSED,
                    "pinned what the suite spent in " + BUDGET.toAbsolutePath()
                            + ". A run that wrote the answer down has not checked it: read the diff, then re-run"
                            + " without -D" + REGENERATE + " to check it.");
            return;
        }
        Map<Kind, Integer> budget = store.readIfThere(BUDGET)
                .map(bytes -> budgetFrom(new String(bytes, StandardCharsets.UTF_8)))
                .orElse(null);
        Verdict verdict = verdictOn(taken, budget, whole);
        write(store, into, verdict.outcome, verdict.said);
    }

    private static void write(Store store, Path into, Verdict.Outcome outcome, String said) {
        store.writeWhole(
                into.resolve("verdict.txt"), (outcome.name() + "\n" + said + "\n").getBytes(StandardCharsets.UTF_8));
    }
}
