package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.Map;
import java.util.regex.Pattern;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

public final class SimAssets {
    private static final Map<String, String> TYPES = Map.of(
            ".glb", "model/gltf-binary",
            ".png", "image/png",
            ".js", "text/javascript; charset=utf-8",
            ".css", "text/css; charset=utf-8");

    private static final Pattern NAME =
            Pattern.compile("[A-Za-z0-9_-]+(\\.[A-Za-z0-9_-]+)*" + "(/[A-Za-z0-9_-]+(\\.[A-Za-z0-9_-]+)*)*");

    private SimAssets() {}

    public static Response serveUnder(Path under, Store store, String name) {
        if (name == null || !NAME.matcher(name).matches()) {
            return Response.error(404, "no such asset");
        }
        String type = typeOf(name);
        if (type == null) {
            return Response.error(404, "no such asset");
        }
        return store.readIfThere(under.resolve(name))
                .map(body -> Response.bytes(type, body))
                .orElseGet(() -> Response.error(404, "no such asset"));
    }

    public static Response serve(String name) {
        if (name == null || !NAME.matcher(name).matches()) {
            return Response.error(404, "no such asset");
        }
        String type = typeOf(name);
        if (type == null) {
            return Response.error(404, "no such asset");
        }
        byte[] body = read(name);
        if (body == null) {
            return Response.error(404, "no such asset");
        }
        return Response.bytes(type, body);
    }

    public static byte[] body(Response response) {
        return response.encoded();
    }

    public static String page(String name) {
        byte[] body = read(name);
        if (body == null) {
            throw new IllegalStateException("missing page " + name + " next to " + SimField.class.getName());
        }
        return new String(body, java.nio.charset.StandardCharsets.UTF_8);
    }

    private static String typeOf(String name) {
        String type = null;
        for (Map.Entry<String, String> kind : TYPES.entrySet()) {
            if (name.endsWith(kind.getKey())) {
                type = kind.getValue();
            }
        }
        return type;
    }

    private static byte[] read(String name) {
        try (InputStream in = SimField.class.getResourceAsStream(name)) {
            return in == null ? null : in.readAllBytes();
        } catch (IOException e) {
            throw new UncheckedIOException("could not read the asset " + name, e);
        }
    }
}
