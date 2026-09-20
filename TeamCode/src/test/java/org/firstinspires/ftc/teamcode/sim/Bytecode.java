package org.firstinspires.ftc.teamcode.sim;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Stream;

/**
 * What the compiled classes mention. A rule about which class may reach something -- a process, a
 * compiler, a password hash -- is held here, over the bytecode, rather than over the sources: an
 * import can be dodged and a fully qualified name cannot.
 */
public final class Bytecode {

    private Bytecode() {}

    /** Every compiled class under a package, from whichever classpath directories hold one. */
    public static List<Path> classesUnder(String packagePath) {
        List<Path> found = new ArrayList<>();
        for (String entry : System.getProperty("java.class.path").split(File.pathSeparator)) {
            Path here = Path.of(entry).resolve(packagePath);
            if (!Files.isDirectory(here)) {
                continue;
            }
            try (Stream<Path> files = Files.walk(here)) {
                files.filter(Files::isRegularFile)
                        .filter(file -> file.toString().endsWith(".class"))
                        .forEach(found::add);
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        }
        return found;
    }

    /** The name of the top-level class a class file belongs to, nested classes folded into it. */
    public static String outermost(Path classFile) {
        String name = classFile.getFileName().toString().replace(".class", "");
        int nested = name.indexOf('$');
        return nested < 0 ? name : name.substring(0, nested);
    }

    /** Whether a class file names something, by its internal name ({@code java/lang/ProcessBuilder}). */
    public static boolean mentions(Path classFile, String reference) {
        try {
            String bytes = new String(Files.readAllBytes(classFile), StandardCharsets.ISO_8859_1);
            return bytes.contains(lengthPrefixed(reference));
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    /** The top-level classes among these that name any of those references. */
    public static Set<String> thatMentionAny(List<Path> classes, List<String> references) {
        Set<String> mentioning = new TreeSet<>();
        for (Path classFile : classes) {
            for (String reference : references) {
                if (mentions(classFile, reference)) {
                    mentioning.add(outermost(classFile));
                }
            }
        }
        return mentioning;
    }

    private static String lengthPrefixed(String reference) {
        return "" + (char) 1 + (char) (reference.length() >> 8) + (char) (reference.length() & 0xff) + reference;
    }
}
