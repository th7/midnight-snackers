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

final class MainSources {
    private static final Path ROOT = Paths.get("src", "main", "java");

    private static MainSources compiled;

    private final Map<String, List<Reference>> byType;

    private final int fileCount;

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
            try {
                files.close();
            } catch (IOException ignored) {
            }
        }
    }

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

    private static Trees treesOf(JavacTask task) {
        try {
            return (Trees) Trees.class
                    .getMethod("instance", javax.tools.JavaCompiler.CompilationTask.class)
                    .invoke(null, task);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("could not get the compiler's trees", e);
        }
    }

    int fileCount() {
        return fileCount;
    }

    List<Reference> referencesTo(String qualifiedTypeName) {
        return byType.getOrDefault(qualifiedTypeName, List.of());
    }

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
