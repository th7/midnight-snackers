package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.util.Map;
import java.util.regex.Pattern;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

/**
 * What the bench serves beside its pages: the field's visual model ({@code field.glb}), the
 * artwork printed on the field, and the renderer that draws them. All of it sits on the classpath
 * next to {@link SimField}, and none of it is text -- a model served through a String comes back
 * from UTF-8 as replacement characters, which is why {@link Response#bytes} exists.
 *
 * <p>The name comes off the URL, so it says what may be asked for rather than trusting what is:
 * a name is a few plain segments, and its last one ends in a kind of file a page actually loads.
 * Everything else on that classpath -- the collision model, the page templates, compiled classes
 * -- sits in the same directory, and a route that served whatever it was asked for would hand out
 * all of it.
 */
public final class SimAssets {
    /** Only what a page loads. The extension decides, so nothing else is reachable by name. */
    private static final Map<String, String> TYPES = Map.of(
            ".glb", "model/gltf-binary",
            ".png", "image/png",
            ".js", "text/javascript; charset=utf-8",
            ".css", "text/css; charset=utf-8");

    /** A name is plain segments: letters, digits, dot, dash, underscore, separated by slashes. */
    private static final Pattern NAME =
            Pattern.compile("[A-Za-z0-9_-]+(\\.[A-Za-z0-9_-]+)*" + "(/[A-Za-z0-9_-]+(\\.[A-Za-z0-9_-]+)*)*");

    private SimAssets() {}

    /** The asset called {@code name}, or 404. */
    public static Response serve(String name) {
        if (name == null || !NAME.matcher(name).matches()) {
            return Response.error(404, "no such asset");
        }
        String type = null;
        for (Map.Entry<String, String> kind : TYPES.entrySet()) {
            if (name.endsWith(kind.getKey())) {
                type = kind.getValue();
            }
        }
        if (type == null) {
            return Response.error(404, "no such asset");
        }
        byte[] body = read(name);
        if (body == null) {
            return Response.error(404, "no such asset");
        }
        return Response.bytes(type, body);
    }

    /** What a response carries, for a test that wants to compare it with the file. */
    public static byte[] body(Response response) {
        return response.encoded();
    }

    /** A page template from the same directory. Pages are text and are not served by name. */
    public static String page(String name) {
        byte[] body = read(name);
        if (body == null) {
            throw new IllegalStateException("missing page " + name + " next to " + SimField.class.getName());
        }
        return new String(body, java.nio.charset.StandardCharsets.UTF_8);
    }

    private static byte[] read(String name) {
        try (InputStream in = SimField.class.getResourceAsStream(name)) {
            return in == null ? null : in.readAllBytes();
        } catch (IOException e) {
            throw new UncheckedIOException("could not read the asset " + name, e);
        }
    }
}
