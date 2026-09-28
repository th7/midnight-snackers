package org.firstinspires.ftc.teamcode.simcore;

import de.thetaphi.forbiddenapis.Checker;
import de.thetaphi.forbiddenapis.ForbiddenApiException;
import de.thetaphi.forbiddenapis.Logger;
import java.io.File;
import java.io.IOException;
import java.io.StringWriter;
import java.io.UncheckedIOException;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Locale;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import javax.tools.Diagnostic;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;
import net.sourceforge.pmd.PMDConfiguration;
import net.sourceforge.pmd.PmdAnalysis;
import net.sourceforge.pmd.lang.LanguageRegistry;
import net.sourceforge.pmd.lang.rule.RulePriority;
import net.sourceforge.pmd.reporting.Report;
import net.sourceforge.pmd.reporting.RuleViolation;

final class Gates {
    private Gates() {}

    static String corePackage() {
        return property("gates.corePackage");
    }

    static final class Verdict {
        final boolean passed;
        final List<String> said;

        Verdict(boolean passed, List<String> said) {
            this.passed = passed;
            this.said = said;
        }

        boolean refusedFor(String reason) {
            return !passed && said.stream().anyMatch(line -> line.contains(reason));
        }

        @Override
        public String toString() {
            return (passed ? "passed" : "refused") + (said.isEmpty() ? "" : ": " + String.join(" | ", said));
        }
    }

    static Verdict javac(Path dir, String className, String source) {
        List<String> options = new ArrayList<>(lines("gates.javac.args"));
        options.addAll(List.of(
                "--release", property("gates.javac.release"), "-processorpath", property("gates.javac.processorPath")));
        return compile(dir, className, source, options);
    }

    static Path classesOf(Path dir, String className, String source) {
        Verdict plain = compile(dir, className, source, List.of("--release", property("gates.javac.release")));
        if (!plain.passed) {
            throw new AssertionError("a breach planted for forbiddenapis has to compile first: " + plain);
        }
        return dir.resolve("classes");
    }

    private static Verdict compile(Path dir, String className, String source, List<String> gates) {
        JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
        if (compiler == null) {
            throw new AssertionError("this JVM has no Java compiler, so the gates cannot be run");
        }
        Path file = write(dir, className, source);
        Path classes = dir.resolve("classes");
        List<String> options = new ArrayList<>(gates);
        options.addAll(List.of("-d", classes.toString(), "-classpath", classpathOr(dir)));
        DiagnosticCollector<JavaFileObject> diagnostics = new DiagnosticCollector<>();
        StringWriter out = new StringWriter();
        boolean ok;
        try (StandardJavaFileManager files =
                compiler.getStandardFileManager(diagnostics, Locale.ROOT, StandardCharsets.UTF_8)) {
            Files.createDirectories(classes);
            ok = compiler.getTask(out, files, diagnostics, options, null, files.getJavaFileObjects(file.toFile()))
                    .call();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        List<String> said = new ArrayList<>();
        for (Diagnostic<? extends JavaFileObject> d : diagnostics.getDiagnostics()) {
            if (d.getKind() != Diagnostic.Kind.NOTE) {
                said.add(d.getKind() + " " + d.getLineNumber() + ": " + d.getMessage(Locale.ROOT));
            }
        }
        if (!out.toString().isBlank()) {
            said.add(out.toString().trim());
        }
        return new Verdict(ok, said);
    }

    private static String classpathOr(Path dir) {
        String classpath = property("gates.javac.classpath");
        if (!classpath.isEmpty()) {
            return classpath;
        }
        try {
            return Files.createDirectories(dir.resolve("nothing")).toString();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    static Verdict pmd(Path dir, String className, String source) {
        Path file = write(dir, className, source);
        PMDConfiguration config = new PMDConfiguration();
        config.setDefaultLanguageVersion(
                LanguageRegistry.PMD.getLanguageById("java").getVersion(property("gates.javac.release")));
        List<String> rules = new ArrayList<>(lines("gates.pmd.ruleSetFiles"));
        rules.addAll(lines("gates.pmd.ruleSets"));
        if (rules.isEmpty()) {
            throw new AssertionError("pmdMain is given no rules, so it refuses nothing");
        }
        rules.forEach(config::addRuleSet);
        config.setMinimumPriority(RulePriority.valueOf(Integer.parseInt(property("gates.pmd.minimumPriority"))));
        config.setIgnoreIncrementalAnalysis(true);
        String classpath = property("gates.javac.classpath");
        if (!classpath.isEmpty()) {
            config.prependAuxClasspath(classpath);
        }
        try (PmdAnalysis pmd = PmdAnalysis.create(config)) {
            if (pmd.getRulesets().isEmpty() || pmd.getReporter().numErrors() > 0) {
                throw new AssertionError("PMD could not load the rules pmdMain is given: " + rules);
            }
            pmd.files().addFile(file);
            Report report = pmd.performAnalysisAndCollectReport();
            if (!report.getProcessingErrors().isEmpty()
                    || !report.getConfigurationErrors().isEmpty()) {
                List<String> errors = new ArrayList<>();
                report.getProcessingErrors().forEach(e -> errors.add(e.getMsg() + "\n" + e.getDetail()));
                report.getConfigurationErrors().forEach(e -> errors.add(e.rule().getName() + ": " + e.issue()));
                throw new AssertionError("PMD could not judge " + className + ": " + errors);
            }
            List<String> said = new ArrayList<>();
            for (RuleViolation violation : report.getViolations()) {
                said.add(violation.getRule().getName() + " " + violation.getBeginLine() + ": "
                        + violation.getDescription());
            }
            return new Verdict(said.isEmpty(), said);
        }
    }

    static Verdict forbiddenApis(Path dir, String className, String source) {
        Path classes = classesOf(dir, className, source);
        List<String> said = new ArrayList<>();
        Logger logger = new Logger() {
            @Override
            public void error(String message) {
                said.add(message);
            }

            @Override
            public void warn(String message) {
                said.add(message);
            }

            @Override
            public void info(String message) {}

            @Override
            public void debug(String message) {}
        };
        try (URLClassLoader loader =
                new URLClassLoader(new URL[] {classes.toUri().toURL()}, ClassLoader.getPlatformClassLoader())) {
            Checker checker = new Checker(
                    logger,
                    loader,
                    EnumSet.of(
                            Checker.Option.FAIL_ON_VIOLATION,
                            Checker.Option.FAIL_ON_MISSING_CLASSES,
                            Checker.Option.FAIL_ON_UNRESOLVABLE_SIGNATURES));
            String target = property("gates.forbidden.target");
            for (String bundled : lines("gates.forbidden.bundled")) {
                checker.addBundledSignatures(bundled, target);
            }
            for (String signatures : lines("gates.forbidden.signaturesFiles")) {
                checker.parseSignaturesFile(new File(signatures));
            }
            for (String signature : lines("gates.forbidden.signatures")) {
                checker.parseSignaturesString(signature);
            }
            if (checker.hasNoSignatures()) {
                throw new AssertionError("forbiddenApisMain is given no signatures, so it refuses nothing");
            }
            try (Stream<Path> walk = Files.walk(classes)) {
                for (Path type :
                        walk.filter(p -> p.toString().endsWith(".class")).collect(Collectors.toList())) {
                    checker.addClassToCheck(type.toFile());
                }
            }
            try {
                checker.run();
                return new Verdict(true, said);
            } catch (ForbiddenApiException e) {
                return new Verdict(false, said);
            }
        } catch (IOException | de.thetaphi.forbiddenapis.ParseException e) {
            throw new AssertionError("forbiddenapis could not load what forbiddenApisMain is given", e);
        }
    }

    static boolean ignoresFailures(String gate) {
        return Boolean.parseBoolean(property("gates." + gate + ".ignoreFailures"));
    }

    private static Path write(Path dir, String className, String source) {
        String packageName = Arrays.stream(source.split("\n"))
                .filter(line -> line.startsWith("package "))
                .map(line -> line.substring("package ".length(), line.indexOf(';')))
                .findFirst()
                .orElse("");
        Path file = dir.resolve("src").resolve(packageName.replace('.', '/')).resolve(className + ".java");
        try {
            Files.createDirectories(file.getParent());
            Files.writeString(file, source);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        return file;
    }

    private static List<String> lines(String name) {
        String value = property(name);
        return value.isEmpty() ? List.of() : List.of(value.split("\n"));
    }

    private static String property(String name) {
        String value = System.getProperty(name);
        if (value == null) {
            throw new AssertionError(name + " is not set: run ./gradlew :simcore:test");
        }
        return value;
    }
}
