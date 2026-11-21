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
 * What the suite spends on the few things that are dear: a child JVM started, a project's sources
 * compiled, a secret put through scrypt, node run, a project tree copied onto disk. Each is counted where it is spent -- inside the one adapter that reaches
 * it -- so the ledger is complete for the same reason nothing else may reach any of them at all,
 * which CostIsCountedWhereItIsSpentTest holds rather than a comment.
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

    public static final String JUDGED_BY = "suiteCost.judgedBy";

    /** The Gradle task whose tests these are, so a missed budget can say which command pins it. */
    public static final String SUITE = "suiteCost.suite";

    /** The budget the build holds this suite to; each suite names a file of its own. */
    public static final String BUDGET = "suiteCost.budget";

    /** A suite, by the task that runs it, and the file that pins what it spends. */
    public record Suite(String task, Path budget) {
        static Suite named() {
            String task = System.getProperty(SUITE);
            String budget = System.getProperty(BUDGET);
            return task == null || budget == null ? null : new Suite(task, Paths.get(budget));
        }

        String pinsAgain() {
            return "./gradlew :TeamCode:" + task + " --rerun -D" + REGENERATE + "=true";
        }
    }

    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    public enum Kind {
        CHILD_JVM("child-jvm", "a child JVM started"),
        COMPILE("compile", "a project's sources compiled"),
        PASSWORD_HASH("password-hash", "a session secret put through scrypt"),
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
                    + " silence: take the line out of the budget, or put the kind back.");
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

    private Cost() {}

    /**
     * The test JVM's agent: the build starts it with this class as one (TeamCode/build.gradle), so
     * its ledger is written up on the way out whatever its tests are. Loading this class arms
     * nothing, so a run whose tests never spend anything counted -- one class, say -- still says
     * what it cost, which is nothing, rather than depending on some test having touched the ledger.
     */
    public static void premain(String ignored) {
        Runtime.getRuntime().addShutdownHook(writingUp(System.getProperty(LEDGER)));
    }

    static Thread writingUp(String ledger) {
        if (ledger == null) {
            throw new IllegalStateException("started as the suite's cost agent with nowhere to write the ledger:"
                    + " the build sets " + LEDGER + " alongside -javaagent, so one without the other is a build"
                    + " that has come apart.");
        }
        return new Thread(() -> writeUp(Paths.get(ledger)), "suite-cost");
    }

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

    public static Verdict verdictOn(Map<Kind, Tally> taken, Map<Kind, Integer> budget, boolean whole, Suite suite) {
        if (!whole) {
            return new Verdict(
                    Verdict.Outcome.COULD_NOT_JUDGE,
                    "this was part of the suite, or the suite split across JVMs, so what it cost could not"
                            + " judge the budget in " + suite.budget() + ": a ledger is one JVM's, and only the"
                            + " whole suite in one JVM spends the whole of what is pinned. Run"
                            + " ./gradlew :TeamCode:" + suite.task() + " to check it.");
        }
        if (budget == null) {
            return new Verdict(
                    Verdict.Outcome.MISSED,
                    "there is no budget at " + suite.budget().toAbsolutePath() + ", so nothing is being checked."
                            + " Pin what the suite spends with " + suite.pinsAgain());
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
            return new Verdict(Verdict.Outcome.MET, "the suite cost what " + suite.budget() + " pins.");
        }
        return new Verdict(
                Verdict.Outcome.MISSED,
                "the suite no longer costs what " + suite.budget() + " pins:\n  " + String.join("\n  ", missed)
                        + "\nA budget is a change detector, not a ceiling. If the change is one you meant, read it"
                        + " and then pin it with " + suite.pinsAgain());
    }

    public static String report(Map<Kind, Tally> taken) {
        StringBuilder out = new StringBuilder("what the suite cost\n");
        for (Kind kind : Kind.values()) {
            Tally tally = taken.get(kind);
            out.append(String.format(
                    Locale.ROOT, "  %-14s %5d %7.1fs   %s%n", kind.key, tally.count, tally.seconds, kind.what));
        }
        out.append("  (a count is pinned in the budget; the time is what was spent\n");
        out.append("   inside the call, so a child JVM's own loading is in its run's time, not here)\n");
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
            Kind kind = Kind.named(entry.getKey());
            budget.put(kind, entry.getValue().getAsInt());
        }
        return budget;
    }

    /**
     * The whole of what the test JVM says about its spending: the ledger it took, the report a
     * person reads, and the verdict the build fails on. Written once, as the JVM goes.
     */
    static void writeUp(Path into) {
        writeUp(
                into,
                Suite.named(),
                Boolean.parseBoolean(System.getProperty(WHOLE, "false")),
                Boolean.getBoolean(REGENERATE));
    }

    static void writeUp(Path into, Suite suite, boolean whole, boolean regenerate) {
        Map<Kind, Tally> taken = taken();
        RepoFile.write(into.resolve("ledger.json"), budgetJson(taken).getBytes(StandardCharsets.UTF_8));
        RepoFile.write(into.resolve("report.txt"), report(taken).getBytes(StandardCharsets.UTF_8));
        if (suite == null) {
            write(
                    into,
                    Verdict.Outcome.MISSED,
                    "the build named no suite (" + SUITE + ") or no budget (" + BUDGET + ") for these tests, so"
                            + " nothing is being checked. TeamCode/build.gradle names both for each suite.");
            return;
        }
        Path budgetFile = suite.budget();
        if (whole && regenerate) {
            RepoFile.write(budgetFile, budgetJson(taken).getBytes(StandardCharsets.UTF_8));
            write(
                    into,
                    Verdict.Outcome.MISSED,
                    "pinned what the suite spent in " + budgetFile.toAbsolutePath()
                            + ". A run that wrote the answer down has not checked it: read the diff, then re-run"
                            + " without -D" + REGENERATE + " to check it.");
            return;
        }
        Map<Kind, Integer> budget = new OnDiskStore()
                .readIfThere(budgetFile)
                .map(bytes -> budgetFrom(new String(bytes, StandardCharsets.UTF_8)))
                .orElse(null);
        Verdict verdict = verdictOn(taken, budget, whole, suite);
        write(into, verdict.outcome, verdict.said);
    }

    private static void write(Path into, Verdict.Outcome outcome, String said) {
        RepoFile.write(
                into.resolve("verdict.txt"), (outcome.name() + "\n" + said + "\n").getBytes(StandardCharsets.UTF_8));
    }
}
