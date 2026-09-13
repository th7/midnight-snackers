package org.firstinspires.ftc.teamcode.sim;

import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.IdentifierTree;
import com.sun.source.tree.LineMap;
import com.sun.source.tree.MemberReferenceTree;
import com.sun.source.tree.MemberSelectTree;
import com.sun.source.tree.Tree;
import com.sun.source.util.JavacTask;
import com.sun.source.util.SourcePositions;
import com.sun.source.util.TreePath;
import com.sun.source.util.TreePathScanner;
import com.sun.source.util.Trees;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.PackageElement;
import javax.lang.model.element.TypeElement;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;

/**
 * Go to definition and find usages for the main sources, answered by the JDK's own compiler:
 * the same kind of task that {@link SimBuild} runs, stopped after {@code analyze()} instead of
 * writing classes, and asked through the {@code com.sun.source} Trees API which declaration the
 * symbol under a position is and where every reference to it sits. No language server and no
 * extra process. The analysis is kept between questions and redone when the sources change,
 * noticed by the same fingerprint the build uses. A file that does not compile still answers
 * for the parts that do.
 * <p>
 * Files are named relative to the source root with {@code /} separators. Lines and columns are
 * 1-based and count characters, so a tab is one column, the way an editor counts.
 */
public final class SourceNavigator {
    /** A place in the sources: the file, its 1-based line and character column, and the line's text. */
    public static final class Location {
        public final String file;
        public final int line;
        public final int column;
        public final String text;

        Location(String file, int line, int column, String text) {
            this.file = file;
            this.line = line;
            this.column = column;
            this.text = text;
        }

        @Override
        public String toString() {
            return file + ":" + line + ":" + column;
        }
    }

    /** What is under a position: a name, a kind, and where it is declared when that is in the sources. */
    public static final class Symbol {
        public final String name;
        /** {@code class}, {@code method}, {@code field}, {@code local variable}, and the other element kinds, in words. */
        public final String kind;
        /** Null when the symbol comes from outside the sources: the SDK, the JDK. */
        public final Location definition;

        Symbol(String name, String kind, Location definition) {
            this.name = name;
            this.kind = kind;
            this.definition = definition;
        }
    }

    /** A symbol and every reference to it in the sources, never its declaration, sorted by file, line, column. */
    public static final class Usages {
        public final Symbol symbol;
        public final List<Location> usages;

        Usages(Symbol symbol, List<Location> usages) {
            this.symbol = symbol;
            this.usages = usages;
        }
    }

    /** The tree kinds whose element is the thing a user means when the cursor is on them. */
    private static final Set<Tree.Kind> NAMED = Set.of(
            Tree.Kind.IDENTIFIER, Tree.Kind.MEMBER_SELECT, Tree.Kind.MEMBER_REFERENCE,
            Tree.Kind.VARIABLE, Tree.Kind.METHOD,
            Tree.Kind.CLASS, Tree.Kind.INTERFACE, Tree.Kind.ENUM, Tree.Kind.ANNOTATION_TYPE,
            Tree.Kind.TYPE_PARAMETER);

    /** One analysis of the whole tree, kept until a source changes. */
    private static final class Analysis {
        final Trees trees;
        final SourcePositions positions;
        /** By source-root-relative key, in key order. */
        final Map<String, CompilationUnitTree> units = new LinkedHashMap<>();
        final Map<String, String> texts = new LinkedHashMap<>();
        final List<String> files = new ArrayList<>();
        final StandardJavaFileManager fileManager;

        Analysis(Trees trees, SourcePositions positions, StandardJavaFileManager fileManager) {
            this.trees = trees;
            this.positions = positions;
            this.fileManager = fileManager;
        }
    }

    private final Path sourceRoot;
    private String lastFingerprint;
    private Analysis analysis;

    /** @param sourceRoot the package root, e.g. {@code TeamCode/src/main/java} */
    public SourceNavigator(Path sourceRoot) {
        this.sourceRoot = sourceRoot.toAbsolutePath().normalize();
    }

    /** Every source file's key, sorted. */
    public synchronized List<String> files() {
        return List.copyOf(analysis().files);
    }

    /** The symbol under the position, or null when there is none there or no such file. */
    public synchronized Symbol definition(String file, int line, int column) {
        Analysis analysis = analysis();
        Element element = elementAt(analysis, file, line, column);
        return element == null ? null : symbolOf(analysis, element);
    }

    /** The symbol under the position and every reference to it in the sources, or null when there is none there. */
    public synchronized Usages usages(String file, int line, int column) {
        Analysis analysis = analysis();
        Element target = elementAt(analysis, file, line, column);
        if (target == null) {
            return null;
        }
        List<Location> usages = new ArrayList<>();
        for (Map.Entry<String, CompilationUnitTree> entry : analysis.units.entrySet()) {
            String key = entry.getKey();
            CompilationUnitTree unit = entry.getValue();
            new TreePathScanner<Void, Void>() {
                @Override
                public Void visitIdentifier(IdentifierTree node, Void p) {
                    note(node, node.getName().toString());
                    return super.visitIdentifier(node, p);
                }

                @Override
                public Void visitMemberSelect(MemberSelectTree node, Void p) {
                    note(node, node.getIdentifier().toString());
                    return super.visitMemberSelect(node, p);
                }

                @Override
                public Void visitMemberReference(MemberReferenceTree node, Void p) {
                    note(node, node.getName().toString());
                    return super.visitMemberReference(node, p);
                }

                private void note(Tree node, String name) {
                    Element element = analysis.trees.getElement(getCurrentPath());
                    if (element != null && element.equals(target)) {
                        long at = namePosition(analysis, unit, key, node, name);
                        if (at >= 0) {
                            usages.add(locationOf(analysis, unit, key, at));
                        }
                    }
                }
            }.scan(unit, null);
        }
        usages.sort(Comparator.comparing((Location l) -> l.file).thenComparingInt(l -> l.line).thenComparingInt(l -> l.column));
        return new Usages(symbolOf(analysis, target), usages);
    }

    // --- what is where ---

    /** The element of the innermost named tree covering the position, or null. */
    private static Element elementAt(Analysis analysis, String file, int line, int column) {
        CompilationUnitTree unit = analysis.units.get(file);
        if (unit == null || line < 1 || column < 1) {
            return null;
        }
        long offset;
        try {
            offset = unit.getLineMap().getStartPosition(line) + (column - 1);
        } catch (IndexOutOfBoundsException e) {
            return null;
        }
        TreePath[] deepest = new TreePath[1];
        new TreePathScanner<Void, Void>() {
            @Override
            public Void scan(Tree tree, Void p) {
                if (tree == null) {
                    return null;
                }
                long start = analysis.positions.getStartPosition(unit, tree);
                long end = analysis.positions.getEndPosition(unit, tree);
                if (start >= 0 && end >= 0 && start <= offset && offset < end) {
                    // only trees covering the position are entered; ancestors are visited before
                    // their descendants, so the last one remembered is the deepest
                    super.scan(tree, p);
                }
                return null;
            }

            @Override
            public Void visitIdentifier(IdentifierTree node, Void p) { return remember(); }

            @Override
            public Void visitMemberSelect(MemberSelectTree node, Void p) { remember(); return super.visitMemberSelect(node, p); }

            @Override
            public Void visitMemberReference(MemberReferenceTree node, Void p) { remember(); return super.visitMemberReference(node, p); }

            @Override
            public Void visitVariable(com.sun.source.tree.VariableTree node, Void p) { remember(); return super.visitVariable(node, p); }

            @Override
            public Void visitMethod(com.sun.source.tree.MethodTree node, Void p) { remember(); return super.visitMethod(node, p); }

            @Override
            public Void visitClass(com.sun.source.tree.ClassTree node, Void p) { remember(); return super.visitClass(node, p); }

            @Override
            public Void visitTypeParameter(com.sun.source.tree.TypeParameterTree node, Void p) { remember(); return super.visitTypeParameter(node, p); }

            private Void remember() {
                deepest[0] = getCurrentPath();
                return null;
            }
        }.scan(unit, null);
        TreePath path = deepest[0];
        if (path == null || !NAMED.contains(path.getLeaf().getKind())) {
            return null;
        }
        Element element = analysis.trees.getElement(path);
        if (element == null) {
            return null;
        }
        Tree leaf = path.getLeaf();
        if (!(leaf instanceof IdentifierTree || leaf instanceof MemberSelectTree || leaf instanceof MemberReferenceTree)) {
            // a declaration: only its name means the thing, not its modifiers, type, or body
            String name = element.getKind() == ElementKind.CONSTRUCTOR
                    ? element.getEnclosingElement().getSimpleName().toString() : element.getSimpleName().toString();
            long start = analysis.positions.getStartPosition(unit, leaf);
            long at = wordFrom(analysis.texts.get(file), name, Math.max(0, start));
            if (at < 0 || offset < at || offset >= at + name.length()) {
                return null;
            }
        }
        return element;
    }

    private static Symbol symbolOf(Analysis analysis, Element element) {
        TreePath declaration = analysis.trees.getPath(element);
        Location location = null;
        if (declaration != null) {
            CompilationUnitTree unit = declaration.getCompilationUnit();
            String key = keyOf(analysis, unit);
            if (key != null) {
                long start = analysis.positions.getStartPosition(unit, declaration.getLeaf());
                String name = element.getKind() == ElementKind.CONSTRUCTOR
                        ? element.getEnclosingElement().getSimpleName().toString() : element.getSimpleName().toString();
                long at = wordFrom(analysis.texts.get(key), name, Math.max(0, start));
                location = locationOf(analysis, unit, key, at < 0 ? Math.max(0, start) : at);
            }
        }
        return new Symbol(nameOf(element), kindOf(element), location);
    }

    private static String nameOf(Element element) {
        switch (element.getKind()) {
            case CLASS: case INTERFACE: case ENUM: case ANNOTATION_TYPE: {
                String qualified = ((TypeElement) element).getQualifiedName().toString();
                return qualified.isEmpty() ? element.getSimpleName().toString() : qualified;
            }
            case METHOD: case CONSTRUCTOR:
                return ownerOf(element) + element.toString();
            case FIELD: case ENUM_CONSTANT:
                return ownerOf(element) + element.getSimpleName();
            case PACKAGE:
                return ((PackageElement) element).getQualifiedName().toString();
            default:
                return element.getSimpleName().toString();
        }
    }

    private static String ownerOf(Element element) {
        Element owner = element.getEnclosingElement();
        return owner instanceof TypeElement ? nameOf(owner) + "." : "";
    }

    private static String kindOf(Element element) {
        return element.getKind().name().toLowerCase(Locale.ROOT).replace('_', ' ');
    }

    /** Where a reference's own name starts: an identifier at its start, a selection or reference just before its end. */
    private static long namePosition(Analysis analysis, CompilationUnitTree unit, String key, Tree node, String name) {
        long start = analysis.positions.getStartPosition(unit, node);
        if (node instanceof IdentifierTree) {
            return start;
        }
        long end = analysis.positions.getEndPosition(unit, node);
        String text = analysis.texts.get(key);
        long at = end - name.length();
        if (at >= 0 && at < text.length() && text.startsWith(name, (int) at)) {
            return at;
        }
        return start < 0 ? -1 : wordFrom(text, name, start);
    }

    /** The first whole-word occurrence of {@code name} at or after {@code from}, or -1. */
    private static long wordFrom(String text, String name, long from) {
        Matcher matcher = Pattern.compile("(?<![\\p{L}\\p{N}_$])" + Pattern.quote(name) + "(?![\\p{L}\\p{N}_$])").matcher(text);
        return matcher.find((int) Math.min(from, text.length())) ? matcher.start() : -1;
    }

    private static Location locationOf(Analysis analysis, CompilationUnitTree unit, String key, long offset) {
        LineMap lines = unit.getLineMap();
        int line = (int) lines.getLineNumber(offset);
        int column = (int) (offset - lines.getStartPosition(line)) + 1;
        String text = analysis.texts.get(key);
        int lineStart = (int) lines.getStartPosition(line);
        int lineEnd = text.indexOf('\n', lineStart);
        String lineText = text.substring(lineStart, lineEnd < 0 ? text.length() : lineEnd);
        return new Location(key, line, column, lineText.trim());
    }

    private static String keyOf(Analysis analysis, CompilationUnitTree unit) {
        for (Map.Entry<String, CompilationUnitTree> entry : analysis.units.entrySet()) {
            if (entry.getValue() == unit) {
                return entry.getKey();
            }
        }
        return null;
    }

    /**
     * {@code Trees.instance(task)}, by reflection: the other overload names a class the Android
     * platform jar this compiles against does not have, which stops javac resolving the call.
     */
    private static Trees treesOf(JavacTask task) {
        try {
            return (Trees) Trees.class.getMethod("instance", javax.tools.JavaCompiler.CompilationTask.class).invoke(null, task);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("could not get the compiler's trees", e);
        }
    }

    // --- the analysis, redone when a source changes ---

    private Analysis analysis() {
        List<Path> sources = SimBuild.sourcesUnder(sourceRoot);
        String fingerprint = SimBuild.fingerprintOf(sourceRoot, sources);
        if (fingerprint.equals(lastFingerprint) && analysis != null) {
            return analysis;
        }
        JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
        if (compiler == null) {
            throw new IllegalStateException("this JVM has no Java compiler; run the server on a JDK, not a JRE");
        }
        StandardJavaFileManager files = compiler.getStandardFileManager(null, null, StandardCharsets.UTF_8);
        Analysis made;
        try {
            List<String> options = List.of(
                    "-cp", System.getProperty("java.class.path"),
                    "--release", "17",
                    "-proc:none",
                    "-nowarn",
                    "-encoding", "UTF-8");
            List<java.io.File> sourceFiles = new ArrayList<>();
            for (Path source : sources) {
                sourceFiles.add(source.toFile());
            }
            Iterable<? extends JavaFileObject> units = files.getJavaFileObjectsFromFiles(sourceFiles);
            JavacTask task = (JavacTask) compiler.getTask(null, files, diagnostic -> { }, options, null, units);
            Trees trees = treesOf(task);
            made = new Analysis(trees, trees.getSourcePositions(), files);
            if (!sources.isEmpty()) {
                Iterable<? extends CompilationUnitTree> parsed = task.parse();
                task.analyze();
                for (CompilationUnitTree unit : parsed) {
                    Path path = Path.of(unit.getSourceFile().toUri());
                    String key = sourceRoot.relativize(path.toAbsolutePath().normalize()).toString().replace('\\', '/');
                    made.units.put(key, unit);
                    made.texts.put(key, unit.getSourceFile().getCharContent(true).toString());
                    made.files.add(key);
                }
            }
            made.files.sort(null);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        if (analysis != null) {
            try {
                analysis.fileManager.close();
            } catch (IOException ignored) {
                // the old analysis is being dropped anyway
            }
        }
        analysis = made;
        lastFingerprint = fingerprint;
        return made;
    }
}
