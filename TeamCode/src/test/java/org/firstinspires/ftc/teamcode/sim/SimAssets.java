package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.DrawnField;
import org.firstinspires.ftc.teamcode.simcore.FieldModel;

public final class SimAssets {
    private static final Map<String, String> TYPES = Map.of(
            ".glb", "model/gltf-binary",
            ".png", "image/png",
            ".js", "text/javascript; charset=utf-8",
            ".css", "text/css; charset=utf-8");

    private static final Pattern NAME =
            Pattern.compile("[A-Za-z0-9_-]+(\\.[A-Za-z0-9_-]+)*" + "(/[A-Za-z0-9_-]+(\\.[A-Za-z0-9_-]+)*)*");

    /**
     * The one line a page reads to learn which field to draw when its query string names none. It is
     * an asset like the models it names, and every server serves the copy committed beside them.
     */
    public static final String DEFAULT_RESOLUTION_FILE = "field-default.js";

    private static final Gson GSON = new Gson();

    private SimAssets() {}

    /**
     * What a page imports to learn which field it draws by default, written here rather than kept in
     * the page's own source, so that the resolution a server serves and the one a page draws are the
     * same sentence rather than two that have to be kept saying the same thing.
     */
    public static String defaultResolutionModule(FieldModel drawn, List<DrawnField> drawable) {
        JsonObject said = new JsonObject();
        for (DrawnField one : drawable) {
            said.addProperty(one.model().asked(), one.said());
        }
        return "// Which field a page draws when its query string names no resolution, and what each field it can\n"
                + "// draw is. The copy committed here is what a bench with nothing kept serves.\n"
                + "export const DEFAULT_RESOLUTION = '" + drawn.asked() + "';\n"
                + "export const FIELD_SAID = " + GSON.toJson(said) + ";\n";
    }

    public static Response serveUnder(Path under, Store store, String name) {
        if (name == null || !NAME.matcher(name).matches()) {
            return Response.error(404, "no such asset");
        }
        String type = typeOf(name);
        if (type == null) {
            return Response.error(404, "no such asset");
        }
        return store.readIfThere(under.resolve(name))
                .map(body -> Response.bytes(type, body).revalidated())
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
        return Response.bytes(type, body).revalidated();
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
