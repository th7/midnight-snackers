package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

/**
 * What a bench builds and starts, in a test of the bench itself: no project on disk, no compile, no
 * classes. It answers what it was told to answer, and hands the child it is given whatever the
 * bench asks of it, so everything the bench does with a build and with a child is exercised and
 * none of it is paid for.
 *
 * <p>What a real build does with real sources is {@code SimBuildTest}'s; what a real child prints
 * is {@code SimChildTest}'s. This is the seam between them.
 */
public final class FakeSources implements SimSources {

    private static final Path CLASSES = Paths.get("classes-that-were-never-built");

    private final SimCatalog fixed;
    private final boolean listedByTheChild;
    private SimBench.BuildFailed willNotBuild;
    private SimBuild.Result check;
    private SimCatalog listed;

    private FakeSources(SimCatalog fixed, boolean listedByTheChild) {
        this.fixed = fixed;
        this.listedByTheChild = listedByTheChild;
    }

    /** Sources whose catalog is known without asking anything. */
    public static FakeSources listing(SimCatalog catalog) {
        return new FakeSources(catalog, false);
    }

    /** Sources whose catalog comes from the child, as a project's does, without a build. */
    public static FakeSources askingTheChild() {
        return new FakeSources(null, true);
    }

    /** Sources that will not build, so a catalog and a run both come back as the build's failure. */
    public FakeSources thatWillNotBuild(String diagnostics) {
        willNotBuild = new SimBench.BuildFailed(diagnostics);
        check = new SimBuild.Result(null, List.of(new SimBuild.Problem("", 0, diagnostics)), true);
        return this;
    }

    private void built() {
        if (willNotBuild != null) {
            throw new SimBench.BuildFailed(willNotBuild.getMessage());
        }
    }

    @Override
    public SimCatalog catalog(Child children) {
        built();
        if (!listedByTheChild) {
            return fixed;
        }
        listed = SimBench.listOn(children, CLASSES);
        return listed;
    }

    @Override
    public Optional<SimCatalog> known() {
        return Optional.ofNullable(listedByTheChild ? listed : fixed);
    }

    @Override
    public Optional<SimBuild.Result> check() {
        return Optional.ofNullable(check);
    }

    @Override
    public Optional<Path> sourceRoot() {
        return Optional.empty();
    }

    @Override
    public List<String> classNames() {
        return fixed == null ? List.of() : fixed.sources();
    }

    @Override
    public Child.Running start(Child children, Consumer<String> log, String[] args) {
        built();
        return children.onThisClasspath(log, args);
    }
}
