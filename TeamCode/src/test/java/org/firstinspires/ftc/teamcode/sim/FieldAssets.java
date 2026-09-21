package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

public final class FieldAssets {
    public static final String FIELD_GLB = "field.glb";
    public static final String TEXTURES_UNDER = "textures";

    /** The assembly as Onshape exported it, kept so a build need not fetch it again. */
    public static final String EXPORT_FILE = "export.gltf";

    /**
     * The one line a page reads to learn which field to draw when its query string names none. It is
     * an asset like the models it names, so a coding server serves it from the setting on its admin
     * page and every other server serves the copy committed beside them.
     */
    public static final String DEFAULT_RESOLUTION_FILE = "field-default.js";

    /**
     * How much of the CAD a model is, in three steps, each adding one thing to the one below it.
     * <b>Low</b> is the field as it plays: the hardware dropped, snapped to a twentieth of an inch,
     * a colour to a part. <b>Medium</b> is those same parts at the points and in the materials the
     * CAD gave them. <b>High</b> adds back every part the export holds, hardware and all. High is
     * what a page draws unless it asks for less or an admin set another: a field that does not look
     * like the field is the thing worth avoiding, and whoever cannot afford it is the one who knows
     * that -- which is why which one the pages draw is a setting on the admin page.
     */
    public enum Resolution {
        LOW("low", FIELD_GLB, FieldGlb.GRID_IN, FieldGlb.Keep.WHAT_WE_DRAW, FieldGlb.Shading.A_COLOUR_APIECE),
        MEDIUM(
                "medium",
                "field-medium.glb",
                FieldGlb.NO_GRID,
                FieldGlb.Keep.WHAT_WE_DRAW,
                FieldGlb.Shading.AS_THE_CAD_DREW_IT),
        HIGH("high", "field-high.glb", FieldGlb.NO_GRID, FieldGlb.Keep.EVERY_PART, FieldGlb.Shading.AS_THE_CAD_DREW_IT);

        /**
         * What a page draws, what a server builds for itself, and what the admin page offers first,
         * until an admin sets another on the admin page. A server holds what was set; this is the
         * answer it holds until then, and the one every server with no admin to ask gives.
         */
        public static final Resolution DEFAULT = HIGH;

        public final String asked;
        public final String file;
        public final double grid;
        public final FieldGlb.Keep keep;
        public final FieldGlb.Shading shading;

        Resolution(String asked, String file, double grid, FieldGlb.Keep keep, FieldGlb.Shading shading) {
            this.asked = asked;
            this.file = file;
            this.grid = grid;
            this.keep = keep;
            this.shading = shading;
        }

        static Resolution named(String asked) {
            for (Resolution one : values()) {
                if (one.asked.equals(asked)) {
                    return one;
                }
            }
            return null;
        }
    }

    /**
     * What a page imports to learn which field it draws by default, written here rather than kept in
     * the page's own source, so that the resolution a server serves and the one a page draws are the
     * same sentence rather than two that have to be kept saying the same thing.
     */
    public static String defaultResolutionModule(Resolution drawn) {
        return "// Which field a page draws when its query string names no resolution. A coding server writes this\n"
                + "// line from the setting on its admin page; the copy committed here is what every other server\n"
                + "// serves, and what a coding server serves until an admin says otherwise.\n"
                + "export const DEFAULT_RESOLUTION = '" + drawn.asked + "';\n";
    }

    /**
     * The resolutions a build was asked for, which is one of them, or all three, or -- asked for
     * nothing in particular -- the one the pages of whoever is asking draw. What that is is the
     * server's to say rather than this class's, so it is named at the call.
     */
    public static List<Resolution> resolutionsNamed(String asked, Resolution whenUnsaid) {
        if (asked == null || asked.isBlank()) {
            return List.of(whenUnsaid);
        }
        if ("all".equals(asked)) {
            return List.of(Resolution.values());
        }
        Resolution one = Resolution.named(asked);
        if (one == null) {
            throw new NotAnAsset("no such resolution: " + asked + ". It is low, medium, high or all.");
        }
        return List.of(one);
    }

    /**
     * The one a page draws, by the name an admin sets it under. A page draws one field, so `all` is
     * not one of them here: it is a thing to ask a build for, not a thing to draw.
     */
    public static Resolution theOneNamed(String asked) {
        Resolution one = asked == null ? null : Resolution.named(asked);
        if (one == null) {
            throw new NotAnAsset("no such resolution: " + asked + ". A page draws low, medium or high.");
        }
        return one;
    }

    public static final List<String> TEXTURES = List.of(
            "BIOBUZZ_Panel_Resized.png",
            "GoalAprilTag_blueaudience.png",
            "GoalAprilTag_bluescoring.png",
            "GoalAprilTag_redaudience.png",
            "GoalAprilTag_redscoring.png");

    private static final byte[] PNG = {(byte) 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    private static final Gson GSON = new Gson();

    private FieldAssets() {}

    /** What a server needs before a page draws what it asks for: the model it draws, and the art. */
    public static List<String> everyAsset(Resolution drawn) {
        List<String> out = new ArrayList<>();
        out.add(drawn.file);
        for (String texture : TEXTURES) {
            out.add(TEXTURES_UNDER + "/" + texture);
        }
        return out;
    }

    public static class NotAnAsset extends RuntimeException {
        public NotAnAsset(String message) {
            super(message);
        }
    }

    /** A build asked for before anything was downloaded: the one refusal that is the asker's to fix. */
    public static final class NothingDownloaded extends NotAnAsset {
        public NothingDownloaded(String message) {
            super(message);
        }
    }

    public static final class Refreshed {
        public final Map<String, Integer> written;

        Refreshed(Map<String, Integer> written) {
            this.written = Map.copyOf(written);
        }

        public int bytes() {
            return written.values().stream().mapToInt(Integer::intValue).sum();
        }

        @Override
        public String toString() {
            return written.size() + " assets, " + bytes() + " bytes";
        }
    }

    /** The dear half and then the cheap one. What the admin page does as two asks, in one. */
    public static Refreshed refresh(Onshape onshape, Store store, Path into, List<Resolution> resolutions) {
        Map<String, Integer> written = new LinkedHashMap<>(download(onshape, store, into).written);
        written.putAll(build(store, into, resolutions).written);
        return new Refreshed(written);
    }

    /**
     * Fetches the assembly and the textures and keeps them. The export is the dear part -- tens of
     * seconds and eleven megabytes -- so it is written whole, and every build after this one reads it
     * from there.
     */
    public static Refreshed download(Onshape onshape, Store store, Path into) {
        Map<String, byte[]> fetched = new LinkedHashMap<>();
        fetched.put(EXPORT_FILE, export(onshape));
        fetched.putAll(textures(onshape));
        return writeAll(store, into, fetched);
    }

    /**
     * Builds each detail from the export already on disk. It is handed no Onshape and so cannot fetch
     * one: a build that found nothing downloaded has to say so rather than quietly reaching for the
     * network, and javac is what holds that rather than a comment.
     */
    public static Refreshed build(Store store, Path into, List<Resolution> resolutions) {
        byte[] export = store.readIfThere(into.resolve(EXPORT_FILE))
                .orElseThrow(() -> new NothingDownloaded(
                        "nothing has been downloaded yet, so there is no export to build from. Download from "
                                + "Onshape first; the build is then under a second."));
        Map<String, byte[]> built = new LinkedHashMap<>();
        for (Resolution resolution : resolutions) {
            built.put(
                    resolution.file,
                    FieldGlb.build(
                            export, SimPlacement.FIELD_SIZE_IN, resolution.grid, resolution.keep, resolution.shading));
        }
        return writeAll(store, into, built);
    }

    private static Refreshed writeAll(Store store, Path into, Map<String, byte[]> assets) {
        Map<String, Integer> written = new LinkedHashMap<>();
        for (Map.Entry<String, byte[]> asset : assets.entrySet()) {
            store.writeWhole(into.resolve(asset.getKey()), asset.getValue());
            written.put(asset.getKey(), asset.getValue().length);
        }
        return new Refreshed(written);
    }

    static byte[] export(Onshape onshape) {
        String url = "/api/v10/assemblies/d/" + Onshape.FIELD_DOCUMENT + "/w/" + Onshape.FIELD_WORKSPACE + "/e/"
                + Onshape.FIELD_ASSEMBLY + "/gltf";
        return onshape.get(url, "model/gltf+json");
    }

    static Map<String, byte[]> textures(Onshape onshape) {
        Map<String, String> available = blobs(onshape);
        List<String> absent = new ArrayList<>();
        for (String name : TEXTURES) {
            if (!available.containsKey(name)) {
                absent.add(name);
            }
        }
        if (!absent.isEmpty()) {
            throw new NotAnAsset("the field document has no blob named " + String.join(", ", absent)
                    + ". It holds: "
                    + (available.isEmpty() ? "(nothing)" : String.join(", ", new TreeSet<>(available.keySet()))));
        }

        Map<String, byte[]> fetched = new LinkedHashMap<>();
        for (String name : TEXTURES) {
            byte[] body = onshape.get(
                    "/api/v10/blobelements/d/" + Onshape.FIELD_DOCUMENT + "/w/" + Onshape.FIELD_WORKSPACE + "/e/"
                            + available.get(name),
                    "application/octet-stream");
            if (body == null || body.length == 0) {
                throw new NotAnAsset(name + " came back empty");
            }
            if (!startsWithPng(body)) {
                throw new NotAnAsset(name + " is not a PNG; it begins "
                        + new String(body, 0, Math.min(16, body.length), StandardCharsets.ISO_8859_1));
            }
            fetched.put(TEXTURES_UNDER + "/" + safeName(name), body);
        }
        return fetched;
    }

    private static Map<String, String> blobs(Onshape onshape) {
        byte[] body = onshape.get(
                "/api/v10/documents/d/" + Onshape.FIELD_DOCUMENT + "/w/" + Onshape.FIELD_WORKSPACE + "/elements");
        JsonArray listed = GSON.fromJson(new String(body, StandardCharsets.UTF_8), JsonArray.class);
        Map<String, String> out = new LinkedHashMap<>();
        if (listed == null) {
            return out;
        }
        for (JsonElement element : listed) {
            JsonObject one = element.getAsJsonObject();
            if (one.has("elementType") && "BLOB".equals(one.get("elementType").getAsString())) {
                out.put(one.get("name").getAsString(), one.get("id").getAsString());
            }
        }
        return out;
    }

    static String safeName(String name) {
        if (name == null || name.isEmpty() || !name.equals(name.trim())) {
            throw new NotAnAsset("blank or padded element name: " + name);
        }
        if (name.equals(".")
                || name.equals("..")
                || name.contains("/")
                || name.contains("\\")
                || name.startsWith("~")
                || name.contains(":")) {
            throw new NotAnAsset("element name is a path rather than a name: " + name);
        }
        return name;
    }

    private static boolean startsWithPng(byte[] body) {
        if (body.length < PNG.length) {
            return false;
        }
        for (int i = 0; i < PNG.length; i++) {
            if (body[i] != PNG[i]) {
                return false;
            }
        }
        return true;
    }
}
