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

    public static final List<String> TEXTURES = List.of(
            "BIOBUZZ_Panel_Resized.png",
            "GoalAprilTag_blueaudience.png",
            "GoalAprilTag_bluescoring.png",
            "GoalAprilTag_redaudience.png",
            "GoalAprilTag_redscoring.png");

    private static final byte[] PNG = {(byte) 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    private static final Gson GSON = new Gson();

    private FieldAssets() {}

    public static final class NotAnAsset extends RuntimeException {
        public NotAnAsset(String message) {
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

    public static Refreshed refresh(Onshape onshape, Store store, Path into) {
        Map<String, byte[]> fetched = new LinkedHashMap<>();
        fetched.put(FIELD_GLB, model(onshape));
        fetched.putAll(textures(onshape));

        Map<String, Integer> written = new LinkedHashMap<>();
        for (Map.Entry<String, byte[]> asset : fetched.entrySet()) {
            store.writeWhole(into.resolve(asset.getKey()), asset.getValue());
            written.put(asset.getKey(), asset.getValue().length);
        }
        return new Refreshed(written);
    }

    static byte[] model(Onshape onshape) {
        String url = "/api/v10/assemblies/d/" + Onshape.FIELD_DOCUMENT + "/w/" + Onshape.FIELD_WORKSPACE + "/e/"
                + Onshape.FIELD_ASSEMBLY + "/gltf";
        byte[] export = onshape.get(url, "model/gltf+json");
        return FieldGlb.build(export, SimPlacement.FIELD_SIZE_IN, FieldGlb.GRID_IN);
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
