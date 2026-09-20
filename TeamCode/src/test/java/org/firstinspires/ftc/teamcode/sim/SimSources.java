package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

public interface SimSources {

    SimCatalog catalog(Child children);

    Optional<SimCatalog> known();

    Optional<SimBuild.Result> check();

    Optional<Path> sourceRoot();

    List<String> classNames();

    Child.Running start(Child children, Consumer<String> log, String[] args);

    static SimSources ofThisClasspath(SimCatalog catalog) {
        return new ThisClasspath(catalog);
    }

    static SimSources ofTheProjectAt(Path project, Path outputDir) {
        Path harnessRoot = project.resolve("TeamCode/src/test/java");
        Path child = harnessRoot.resolve(SimChild.class.getName().replace('.', '/') + ".java");
        if (!Files.isRegularFile(child)) {
            throw new IllegalArgumentException("no simulator in " + project + ": " + child
                    + " is missing, and the child would run this server's simulator instead of the project's own");
        }
        return new AProject(
                new SimBuild(project.resolve("TeamCode/src/main/java"), harnessRoot, outputDir.resolve("classes")));
    }

    final class ThisClasspath implements SimSources {
        private final SimCatalog catalog;

        private ThisClasspath(SimCatalog catalog) {
            this.catalog = catalog;
        }

        @Override
        public SimCatalog catalog(Child children) {
            return catalog;
        }

        @Override
        public Optional<SimCatalog> known() {
            return Optional.of(catalog);
        }

        @Override
        public Optional<SimBuild.Result> check() {
            return Optional.empty();
        }

        @Override
        public Optional<Path> sourceRoot() {
            return Optional.empty();
        }

        @Override
        public List<String> classNames() {
            return catalog.sources();
        }

        @Override
        public Child.Running start(Child children, Consumer<String> log, String[] args) {
            return children.onThisClasspath(log, args);
        }
    }

    final class AProject implements SimSources {
        private final SimBuild build;
        private SimCatalog listed;
        private Path listedFrom;

        private AProject(SimBuild build) {
            this.build = build;
        }

        private Path built() {
            SimBuild.Result result = build.build();
            if (result.classes == null) {
                throw new SimBench.BuildFailed(result.diagnostics);
            }
            return result.classes;
        }

        @Override
        public synchronized SimCatalog catalog(Child children) {
            Path classes = built();
            if (listed != null && classes.equals(listedFrom)) {
                return listed;
            }
            listed = SimBench.listOn(children, classes);
            listedFrom = classes;
            return listed;
        }

        @Override
        public synchronized Optional<SimCatalog> known() {
            return Optional.ofNullable(listed);
        }

        @Override
        public Optional<SimBuild.Result> check() {
            return Optional.of(build.build());
        }

        @Override
        public Optional<Path> sourceRoot() {
            return Optional.of(build.sourceRoot());
        }

        @Override
        public List<String> classNames() {
            return List.of();
        }

        @Override
        public Child.Running start(Child children, Consumer<String> log, String[] args) {
            return children.onTheClassesAt(built(), log, args);
        }
    }
}
