package org.firstinspires.ftc.teamcode;

import com.sun.source.tree.ClassTree;
import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.IdentifierTree;
import com.sun.source.tree.MemberSelectTree;
import com.sun.source.tree.Tree;
import com.sun.source.util.JavacTask;
import com.sun.source.util.SourcePositions;
import com.sun.source.util.TreePath;
import com.sun.source.util.TreePathScanner;
import com.sun.source.util.Trees;
import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.TypeElement;
import javax.tools.Diagnostic;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;
import org.firstinspires.ftc.teamcode.sim.SimBuild;

/**
 * The robot's main sources, asked questions about by javac's own understanding of them.
 *
 * <p>Some rules about this code are not about any one class's behaviour but about what the code as
 * a whole is allowed to mention, and a test is the only thing that can hold them. Reading the
 * sources as text to do it is close enough to work and wrong in both directions: a name in a
 * comment or a string is not a use, and a use through a star import or an inherited name does not
 * carry the name being looked for. So this compiles them, stops after {@code analyze()}, and
 * matches <b>resolved symbols</b> -- the same machinery {@code SourceNavigator} answers the editor
 * with, asked a different question.
 *
 * <p>Compiled once and shared: a rule costs a scan, not a build.
 */
final class MainSources {
    private static final Path ROOT = Paths.get("src", "main", "java");

    private static MainSources compiled;

    /** Every type named in the sources, and everywhere it is named. */
    private final Map<String, List<Reference>> byType;

    private final int fileCount;

    /** Where a type is named in the sources, and what it is named inside. */
    record Reference(String file, long line, String inside) {
        @Override
        public String toString() {
            return file + ":" + line + " in " + inside;
        }
    }

    private MainSources(Map<String, List<Reference>> byType, int fileCount) {
        this.byType = byType;
        this.fileCount = fileCount;
    }

    /**
     * The main sources, compiled. Fails rather than answering from nothing: a rule that looked at
     * no files, or at files that did not compile, has not judged anything and must not look as
     * though it has.
     */
    static synchronized MainSources compiled() {
        if (compiled != null) {
            return compiled;
        }
        if (!Files.isDirectory(ROOT)) {
            throw new IllegalStateException("no main sources at " + ROOT.toAbsolutePath()
                    + "; these rules read the robot code and cannot judge without it");
        }
        List<Path> sources = javaFilesUnder(ROOT);
        if (sources.size() < 10) {
            throw new IllegalStateException("only " + sources.size() + " main sources found under "
                    + ROOT.toAbsolutePath() + "; that is too few to be the robot code, so these rules cannot judge");
        }

        JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
        StandardJavaFileManager files = compiler.getStandardFileManager(null, null, StandardCharsets.UTF_8);
        List<String> errors = new ArrayList<>();
        try {
            List<File> sourceFiles = sources.stream().map(Path::toFile).collect(Collectors.toList());
            Iterable<? extends JavaFileObject> inputs = files.getJavaFileObjectsFromFiles(sourceFiles);
            List<String> options = List.of(
                    "-cp",
                    String.join(File.pathSeparator, SimBuild.libraries()),
                    "--release",
                    "17",
                    "-proc:none",
                    "-nowarn",
                    "-encoding",
                    "UTF-8");
            JavacTask task = (JavacTask) compiler.getTask(
                    null,
                    files,
                    diagnostic -> {
                        if (diagnostic.getKind() == Diagnostic.Kind.ERROR) {
                            errors.add(diagnostic.getSource() + ":" + diagnostic.getLineNumber() + " "
                                    + diagnostic.getMessage(null));
                        }
                    },
                    options,
                    null,
                    inputs);
            Trees trees = treesOf(task);
            List<CompilationUnitTree> parsed = new ArrayList<>();
            task.parse().forEach(parsed::add);
            task.analyze();
            if (!errors.isEmpty()) {
                throw new IllegalStateException("the robot's main sources do not compile, so these rules cannot "
                        + "judge what they mention:\n" + String.join("\n", errors));
            }
            compiled = new MainSources(index(parsed, trees), parsed.size());
            return compiled;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        } finally {
            // The answers are read out above and kept as values, so nothing here is needed again.
            // Holding the compiler's trees and an open file manager for the life of the test JVM
            // would leave sixty parsed files on the heap behind every later test.
            try {
                files.close();
            } catch (IOException ignored) {
                // nothing is read through it again
            }
        }
    }

    /** Reads every name javac resolved to a type out of the trees, once, into plain values. */
    private static Map<String, List<Reference>> index(List<CompilationUnitTree> units, Trees trees) {
        SourcePositions positions = trees.getSourcePositions();
        Map<String, List<Reference>> byType = new HashMap<>();
        for (CompilationUnitTree unit : units) {
            Path path = Path.of(unit.getSourceFile().toUri());
            String file = ROOT.toAbsolutePath()
                    .relativize(path.toAbsolutePath().normalize())
                    .toString()
                    .replace('\\', '/');
            new TreePathScanner<Void, Void>() {
                @Override
                public Void visitIdentifier(IdentifierTree node, Void unused) {
                    record(getCurrentPath(), node);
                    return super.visitIdentifier(node, unused);
                }

                @Override
                public Void visitMemberSelect(MemberSelectTree node, Void unused) {
                    record(getCurrentPath(), node);
                    return super.visitMemberSelect(node, unused);
                }

                private void record(TreePath treePath, Tree node) {
                    Element element = trees.getElement(treePath);
                    if (!(element instanceof TypeElement) || element.getKind() == ElementKind.PACKAGE) {
                        return;
                    }
                    long offset = positions.getStartPosition(unit, node);
                    long line = offset < 0 ? 0 : unit.getLineMap().getLineNumber(offset);
                    byType.computeIfAbsent(
                                    ((TypeElement) element).getQualifiedName().toString(), key -> new ArrayList<>())
                            .add(new Reference(file, line, enclosingTypeOf(treePath)));
                }
            }.scan(unit, null);
        }
        return byType;
    }

    /**
     * {@code Trees.instance(task)}, by reflection: the other overload names a class the Android
     * platform jar this compiles against does not have, which stops javac resolving the call. The
     * same dodge {@code SourceNavigator} needs, for the same reason.
     */
    private static Trees treesOf(JavacTask task) {
        try {
            return (Trees) Trees.class
                    .getMethod("instance", javax.tools.JavaCompiler.CompilationTask.class)
                    .invoke(null, task);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("could not get the compiler's trees", e);
        }
    }

    /** How many source files these rules are reading, so a rule can say it looked at something. */
    int fileCount() {
        return fileCount;
    }

    /**
     * Everywhere {@code qualifiedTypeName} is named: an import of it, a use of its simple name
     * under a star import, a reference through any other name it has. Only names that javac
     * resolved to that very type, so a mention in a comment or a string is not one.
     */
    List<Reference> referencesTo(String qualifiedTypeName) {
        return byType.getOrDefault(qualifiedTypeName, List.of());
    }

    /** The name of the class a reference sits inside, for a message that says where to look. */
    private static String enclosingTypeOf(TreePath path) {
        for (TreePath up = path; up != null; up = up.getParentPath()) {
            if (up.getLeaf() instanceof ClassTree) {
                return ((ClassTree) up.getLeaf()).getSimpleName().toString();
            }
        }
        return "an import";
    }

    private static List<Path> javaFilesUnder(Path directory) {
        try (Stream<Path> files = Files.walk(directory)) {
            return files.filter(path -> path.toString().endsWith(".java")).collect(Collectors.toList());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
