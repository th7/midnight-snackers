package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.util.List;
import java.util.Locale;

final class MergeReport {
    private static final Gson GSON = new Gson();

    enum Op {
        PULL("pull", "pulled"),
        PUSH("push", "pushed");

        final String name;
        final String past;

        Op(String name, String past) {
            this.name = name;
            this.past = past;
        }
    }

    enum Voice {
        USER,
        ADMIN
    }

    enum Severity {
        OK,

        NONE,

        WARN,

        BAD;

        String json() {
            return this == NONE ? "" : name().toLowerCase(Locale.ROOT);
        }
    }

    private final Worktrees.Merge merge;
    private final Op op;
    private final String username;
    private final Voice voice;
    private final String worktreePath;

    private MergeReport(Worktrees.Merge merge, Op op, String username, Voice voice, String worktreePath) {
        this.merge = merge;
        this.op = op;
        this.username = username;
        this.voice = voice;
        this.worktreePath = worktreePath;
    }

    static MergeReport of(Worktrees.Merge merge, Op op, String username, Voice voice, String worktreePath) {
        return new MergeReport(merge, op, username, voice, worktreePath);
    }

    int status() {
        switch (merge.outcome) {
            case MERGED:
            case NOTHING:
                return 200;
            default:
                return 409;
        }
    }

    Severity severity() {
        switch (merge.outcome) {
            case MERGED:
                return remoteFailed() ? Severity.WARN : Severity.OK;
            case NOTHING:
                return remoteFailed() ? Severity.WARN : Severity.NONE;
            case CONFLICTS:
            case REFUSED:
                return Severity.BAD;
            default:
                return Severity.WARN;
        }
    }

    String message() {
        String whose = voice == Voice.USER ? "your" : username + "'s";
        String help = voice == Voice.USER ? "; ask your coach for help" : "";
        switch (merge.outcome) {
            case MERGED:
                return did();
            case NOTHING:
                return nothing();
            case UNCOMMITTED:
                return (voice == Voice.USER ? "" : username + " must ") + "commit first: "
                        + String.join(", ", merge.files);
            case CONFLICTS:
                return whose + " changes conflict with " + Worktrees.DEVELOP + " in " + String.join(", ", merge.files)
                        + help;
            default:
                return "git could not " + op.name + help + ": " + merge.detail;
        }
    }

    private String did() {
        if (op == Op.PULL) {
            return voice == Voice.USER
                    ? "pulled " + Worktrees.DEVELOP
                    : "pulled " + Worktrees.DEVELOP + " into " + username + "'s worktree";
        }
        return "pushed to " + Worktrees.DEVELOP + remoteSuffix()
                + (merge.detail == null
                        ? ""
                        : "; but your worktree is not up to date; commit and pull: " + merge.detail);
    }

    private String nothing() {
        if (op == Op.PULL) {
            return voice == Voice.USER ? "nothing to pull" : "nothing to pull for " + username;
        }
        if (merge.remote != null && merge.remote.outcome == Worktrees.Remote.Outcome.PUSHED) {
            return "nothing new of yours to push; pushed " + Worktrees.DEVELOP + " to " + merge.remote.name;
        }
        return "nothing to push" + (remoteFailed() ? remoteSuffix() : "");
    }

    private String remoteSuffix() {
        if (merge.remote == null) {
            return "";
        }
        switch (merge.remote.outcome) {
            case PUSHED:
                return " and to " + merge.remote.name;
            case UP_TO_DATE:
                return " (" + merge.remote.name + " already had it)";
            default:
                return "; could not push to " + merge.remote.name + ", ask your coach: " + merge.remote.detail;
        }
    }

    private boolean remoteFailed() {
        return merge.remote != null && merge.remote.outcome == Worktrees.Remote.Outcome.FAILED;
    }

    List<String> recipe() {
        if (merge.outcome == Worktrees.Outcome.CONFLICTS) {
            return List.of(
                    "cd " + (worktreePath == null ? "<the user's worktree>" : worktreePath),
                    "git merge " + Worktrees.DEVELOP + "         # resolve the conflicts in an editor",
                    "git add -A && git commit  # then " + username + " presses Push");
        }
        if (remoteFailed()) {
            return List.of("git push " + merge.remote.name + " " + Worktrees.DEVELOP
                    + "   # from the checkout, once the network is back; a rejected push wants git pull first");
        }
        if (merge.outcome == Worktrees.Outcome.REFUSED) {
            return List.of("# git refused a merge it had already called clean: " + merge.detail);
        }
        return List.of();
    }

    JsonObject json() {
        JsonObject reply = new JsonObject();
        reply.addProperty("op", op.name);
        reply.addProperty(
                "outcome",
                merge.outcome == Worktrees.Outcome.MERGED
                        ? op.past
                        : merge.outcome.name().toLowerCase(Locale.ROOT));
        reply.add("files", GSON.toJsonTree(merge.files));
        reply.addProperty("detail", merge.detail);
        reply.addProperty("message", message());
        reply.addProperty("severity", severity().json());
        JsonArray recipe = new JsonArray();
        for (String line : recipe()) {
            recipe.add(line);
        }
        reply.add("recipe", recipe);
        if (merge.remote == null) {
            reply.add("remote", null);
        } else {
            JsonObject remote = new JsonObject();
            remote.addProperty("name", merge.remote.name);
            remote.addProperty("outcome", merge.remote.outcome.json);
            remote.addProperty("detail", merge.remote.detail);
            reply.add("remote", remote);
        }
        return reply;
    }

    JsonObject record(long atMillis) {
        JsonObject record = json();
        record.addProperty("atMillis", atMillis);
        record.addProperty("by", voice.name().toLowerCase(Locale.ROOT));
        return record;
    }
}
