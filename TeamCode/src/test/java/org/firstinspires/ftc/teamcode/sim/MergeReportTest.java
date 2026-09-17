package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

/**
 * How a pull or a push is reported, with no repository, no listener and no browser: the whole of
 * it is a function of the merge, whose it was and who is being told. That is what the module is
 * for -- it used to be a six-parameter method whose callers composed half the prose, so the only
 * way to see what a given outcome said was to make git produce it.
 */
public class MergeReportTest {
    private static final String WORKTREE = "/state/worktrees/checkout/ada";

    private static MergeReport report(Worktrees.Merge merge, MergeReport.Op op, MergeReport.Voice voice) {
        return MergeReport.of(merge, op, "ada", voice, WORKTREE);
    }

    private static Worktrees.Merge merge(Worktrees.Outcome outcome, List<String> files, String detail) {
        return new Worktrees.Merge(outcome, files, detail);
    }

    private static Worktrees.Merge pushed(Worktrees.Outcome outcome, Worktrees.Remote.Outcome remote, String detail) {
        return new Worktrees.Merge(outcome, List.of(), null, new Worktrees.Remote("origin", remote, detail));
    }

    // --- what it says ---

    @Test
    public void aPullThatLandedSpeaksToTheUserOrAboutThemAccordingToTheVoice() {
        Worktrees.Merge merged = merge(Worktrees.Outcome.MERGED, List.of(), null);

        assertEquals(
                "pulled develop",
                report(merged, MergeReport.Op.PULL, MergeReport.Voice.USER).message());
        assertEquals(
                "pulled develop into ada's worktree",
                report(merged, MergeReport.Op.PULL, MergeReport.Voice.ADMIN).message());
    }

    @Test
    public void nothingToPullReadsDifferentlyToTheAdmin() {
        Worktrees.Merge nothing = merge(Worktrees.Outcome.NOTHING, List.of(), null);

        assertEquals(
                "nothing to pull",
                report(nothing, MergeReport.Op.PULL, MergeReport.Voice.USER).message());
        assertEquals(
                "nothing to pull for ada",
                report(nothing, MergeReport.Op.PULL, MergeReport.Voice.ADMIN).message());
    }

    @Test
    public void onlyTheUserIsSentToTheirCoachSinceTheAdminIsTheCoach() {
        Worktrees.Merge conflicts = merge(Worktrees.Outcome.CONFLICTS, List.of("TeamCode/Plans.java"), null);

        assertTrue(report(conflicts, MergeReport.Op.PULL, MergeReport.Voice.USER)
                .message()
                .contains("ask your coach"));
        assertTrue(!report(conflicts, MergeReport.Op.PULL, MergeReport.Voice.ADMIN)
                .message()
                .contains("ask your coach"));
    }

    @Test
    public void aPushThatReachedTheRemoteSaysSoAndOneThatCouldNotSaysThat() {
        assertEquals(
                "pushed to develop and to origin",
                report(
                                pushed(Worktrees.Outcome.MERGED, Worktrees.Remote.Outcome.PUSHED, null),
                                MergeReport.Op.PUSH,
                                MergeReport.Voice.USER)
                        .message());
        assertTrue(report(
                        pushed(Worktrees.Outcome.MERGED, Worktrees.Remote.Outcome.FAILED, "no route to host"),
                        MergeReport.Op.PUSH,
                        MergeReport.Voice.USER)
                .message()
                .contains("could not push to origin, ask your coach: no route to host"));
        assertEquals(
                "pushed to develop (origin already had it)",
                report(
                                pushed(Worktrees.Outcome.MERGED, Worktrees.Remote.Outcome.UP_TO_DATE, null),
                                MergeReport.Op.PUSH,
                                MergeReport.Voice.USER)
                        .message());
    }

    // --- how bad it is: the partition the two pages used to make for themselves, differently ---

    @Test
    public void severityIsDecidedOnceHere() {
        assertEquals(
                MergeReport.Severity.OK,
                report(merge(Worktrees.Outcome.MERGED, List.of(), null), MergeReport.Op.PULL, MergeReport.Voice.USER)
                        .severity());
        assertEquals(
                MergeReport.Severity.NONE,
                report(merge(Worktrees.Outcome.NOTHING, List.of(), null), MergeReport.Op.PULL, MergeReport.Voice.USER)
                        .severity());
        assertEquals(
                MergeReport.Severity.WARN,
                report(
                                merge(Worktrees.Outcome.UNCOMMITTED, List.of("a.java"), null),
                                MergeReport.Op.PUSH,
                                MergeReport.Voice.USER)
                        .severity());
        assertEquals(
                MergeReport.Severity.BAD,
                report(
                                merge(Worktrees.Outcome.CONFLICTS, List.of("a.java"), null),
                                MergeReport.Op.PULL,
                                MergeReport.Voice.USER)
                        .severity());
        assertEquals(
                MergeReport.Severity.BAD,
                report(
                                merge(Worktrees.Outcome.REFUSED, List.of(), "git said no"),
                                MergeReport.Op.PULL,
                                MergeReport.Voice.USER)
                        .severity());
    }

    /** A push that landed but never reached origin is the case the two pages disagreed about. */
    @Test
    public void aLandedPushThatCouldNotReachTheRemoteIsAWarningNotASuccess() {
        assertEquals(
                MergeReport.Severity.WARN,
                report(
                                pushed(Worktrees.Outcome.MERGED, Worktrees.Remote.Outcome.FAILED, "no route"),
                                MergeReport.Op.PUSH,
                                MergeReport.Voice.USER)
                        .severity());
        assertEquals(
                MergeReport.Severity.WARN,
                report(
                                pushed(Worktrees.Outcome.NOTHING, Worktrees.Remote.Outcome.FAILED, "no route"),
                                MergeReport.Op.PUSH,
                                MergeReport.Voice.USER)
                        .severity());
    }

    @Test
    public void theStatusIsTwoHundredWhenSomethingOrNothingHappenedAndFourOhNineOtherwise() {
        assertEquals(
                200,
                report(merge(Worktrees.Outcome.MERGED, List.of(), null), MergeReport.Op.PULL, MergeReport.Voice.USER)
                        .status());
        assertEquals(
                200,
                report(merge(Worktrees.Outcome.NOTHING, List.of(), null), MergeReport.Op.PULL, MergeReport.Voice.USER)
                        .status());
        assertEquals(
                409,
                report(
                                merge(Worktrees.Outcome.CONFLICTS, List.of("a"), null),
                                MergeReport.Op.PULL,
                                MergeReport.Voice.USER)
                        .status());
    }

    // --- the recipe, which used to be written in the admin page's own hand ---

    @Test
    public void aConflictsRecipeNamesTheRealWorktreeAndTheUser() {
        List<String> recipe = report(
                        merge(Worktrees.Outcome.CONFLICTS, List.of("TeamCode/Plans.java"), null),
                        MergeReport.Op.PULL,
                        MergeReport.Voice.ADMIN)
                .recipe();

        assertEquals(3, recipe.size());
        assertEquals("cd " + WORKTREE, recipe.get(0));
        assertTrue(recipe.get(1).startsWith("git merge develop"));
        assertTrue(recipe.get(2).contains("ada presses Push"));
    }

    @Test
    public void aFailedRemotePushRecipeIsTheOneCommandThatFixesIt() {
        List<String> recipe = report(
                        pushed(Worktrees.Outcome.MERGED, Worktrees.Remote.Outcome.FAILED, "no route"),
                        MergeReport.Op.PUSH,
                        MergeReport.Voice.ADMIN)
                .recipe();

        assertEquals(1, recipe.size());
        assertTrue(recipe.get(0).startsWith("git push origin develop"));
    }

    @Test
    public void thereIsNoRecipeWhenNothingIsWrong() {
        assertEquals(
                List.of(),
                report(merge(Worktrees.Outcome.MERGED, List.of(), null), MergeReport.Op.PULL, MergeReport.Voice.USER)
                        .recipe());
        assertEquals(
                List.of(),
                report(
                                merge(Worktrees.Outcome.UNCOMMITTED, List.of("a.java"), null),
                                MergeReport.Op.PUSH,
                                MergeReport.Voice.USER)
                        .recipe());
    }

    // --- the wire ---

    @Test
    public void theReplyCarriesItsOwnVerdictSoNoPageHasToWorkOneOut() {
        var json = report(
                        merge(Worktrees.Outcome.CONFLICTS, List.of("TeamCode/Plans.java"), null),
                        MergeReport.Op.PULL,
                        MergeReport.Voice.USER)
                .json();

        assertEquals("pull", json.get("op").getAsString());
        assertEquals("conflicts", json.get("outcome").getAsString());
        assertEquals("bad", json.get("severity").getAsString());
        assertEquals(3, json.getAsJsonArray("recipe").size());
    }

    @Test
    public void aMergedPullIsReportedInThePastTense() {
        assertEquals(
                "pulled",
                report(merge(Worktrees.Outcome.MERGED, List.of(), null), MergeReport.Op.PULL, MergeReport.Voice.USER)
                        .json()
                        .get("outcome")
                        .getAsString());
        assertEquals(
                "pushed",
                report(merge(Worktrees.Outcome.MERGED, List.of(), null), MergeReport.Op.PUSH, MergeReport.Voice.USER)
                        .json()
                        .get("outcome")
                        .getAsString());
    }

    @Test
    public void theRecordIsTheReplyStampedWithWhenAndWhoAskedForIt() {
        var record = report(
                        merge(Worktrees.Outcome.MERGED, List.of(), null), MergeReport.Op.PULL, MergeReport.Voice.ADMIN)
                .record(1234L);

        assertEquals(1234L, record.get("atMillis").getAsLong());
        assertEquals("admin", record.get("by").getAsString());
        assertEquals("pulled", record.get("outcome").getAsString());
    }
}
