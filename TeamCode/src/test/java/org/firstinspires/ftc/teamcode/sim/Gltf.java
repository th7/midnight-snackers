package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Base64;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

public final class Gltf {
    private static final byte[] MAGIC = {'g', 'l', 'T', 'F'};
    private static final int VERSION = 2;
    private static final int CHUNK_JSON = 0x4E4F534A;
    private static final int CHUNK_BIN = 0x004E4942;
    private static final int TRIANGLES = 4;
    private static final int ARRAY_BUFFER = 34962;
    private static final int ELEMENT_ARRAY_BUFFER = 34963;
    private static final int UNSIGNED_SHORT = 5123;
    private static final int UNSIGNED_INT = 5125;
    private static final int FLOAT = 5126;

    private static final Map<Integer, Integer> COMPONENT_WIDTHS =
            Map.of(5120, 1, 5121, 1, 5122, 2, 5123, 2, 5125, 4, 5126, 4);
    private static final Map<String, Integer> TYPE_COUNTS =
            Map.of("SCALAR", 1, "VEC2", 2, "VEC3", 3, "VEC4", 4, "MAT4", 16);

    private static final Gson GSON = new Gson();

    private Gltf() {}

    public static final class NotGltf extends RuntimeException {
        public NotGltf(String message) {
            super(message);
        }
    }

    public static final class Part {
        public final String name;
        public final List<String> path;
        public final double[] triangles;
        public final String colour;

        public Part(String name, List<String> path, double[] triangles, String colour) {
            this.name = name;
            this.path = List.copyOf(path);
            this.triangles = triangles;
            this.colour = colour;
        }

        public int count() {
            return triangles.length / 9;
        }

        @Override
        public String toString() {
            return "<Part " + (name == null || name.isEmpty() ? "(unnamed)" : name) + " " + count() + " triangles>";
        }
    }

    private static final class Document {
        final JsonObject json;
        final List<byte[]> buffers;

        Document(JsonObject json, List<byte[]> buffers) {
            this.json = json;
            this.buffers = buffers;
        }
    }

    public static List<Part> read(byte[] data) {
        Document document = parse(data);
        JsonArray nodes = array(document.json, "nodes");
        JsonArray meshes = array(document.json, "meshes");
        JsonArray scenes = array(document.json, "scenes");
        int scene = document.json.has("scene") ? document.json.get("scene").getAsInt() : 0;
        if (scenes.size() == 0) {
            throw new NotGltf("the glTF has no scene");
        }
        if (scene < 0 || scene >= scenes.size()) {
            throw new NotGltf("the glTF names scene " + scene + ", which does not exist");
        }

        List<Part> parts = new ArrayList<>();
        Set<Integer> seen = new HashSet<>();
        for (JsonElement root : array(scenes.get(scene).getAsJsonObject(), "nodes")) {
            walk(document, nodes, meshes, root.getAsInt(), identity(), List.of(), seen, parts);
        }
        return parts;
    }

    private static void walk(
            Document document,
            JsonArray nodes,
            JsonArray meshes,
            int index,
            double[] parent,
            List<String> path,
            Set<Integer> seen,
            List<Part> parts) {
        if (index < 0 || index >= nodes.size()) {
            throw new NotGltf("a node names child " + index + ", which does not exist");
        }
        if (!seen.add(index)) {
            throw new NotGltf("node " + index + " is reached twice; the scene is not a tree");
        }
        JsonObject node = nodes.get(index).getAsJsonObject();
        double[] matrix = multiply(parent, placement(node));
        String name = node.has("name") ? node.get("name").getAsString() : "";
        List<String> below = name.isEmpty() ? path : append(path, name);

        if (node.has("mesh")) {
            int which = node.get("mesh").getAsInt();
            if (which < 0 || which >= meshes.size()) {
                throw new NotGltf("node " + (name.isEmpty() ? String.valueOf(index) : name)
                        + " names a mesh that does not exist");
            }
            for (JsonElement primitive : array(meshes.get(which).getAsJsonObject(), "primitives")) {
                parts.add(partOf(document, primitive.getAsJsonObject(), name, below, matrix));
            }
        }
        for (JsonElement child : array(node, "children")) {
            walk(document, nodes, meshes, child.getAsInt(), matrix, below, seen, parts);
        }
    }

    private static Part partOf(
            Document document, JsonObject primitive, String name, List<String> path, double[] matrix) {
        String called = name.isEmpty() ? "(unnamed)" : name;
        int mode = primitive.has("mode") ? primitive.get("mode").getAsInt() : TRIANGLES;
        if (mode != TRIANGLES) {
            throw new NotGltf("primitive of " + called + " has mode " + mode + "; a solid export is triangles");
        }
        JsonObject attributes =
                primitive.has("attributes") ? primitive.getAsJsonObject("attributes") : new JsonObject();
        if (!attributes.has("POSITION")) {
            throw new NotGltf("primitive of " + called + " has no POSITION");
        }
        double[] points = vectors(document, attributes.get("POSITION").getAsInt(), 3);
        int howMany = points.length / 3;
        for (int i = 0; i < howMany; i++) {
            double x = points[i * 3];
            double y = points[i * 3 + 1];
            double z = points[i * 3 + 2];
            points[i * 3] = matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12];
            points[i * 3 + 1] = matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13];
            points[i * 3 + 2] = matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14];
        }

        int[] order;
        if (primitive.has("indices")) {
            double[] read = vectors(document, primitive.get("indices").getAsInt(), 1);
            order = new int[read.length];
            for (int i = 0; i < read.length; i++) {
                order[i] = (int) read[i];
            }
        } else {
            order = new int[howMany];
            for (int i = 0; i < howMany; i++) {
                order[i] = i;
            }
        }
        if (order.length % 3 != 0) {
            throw new NotGltf(
                    "primitive of " + called + " has " + order.length + " vertices, which is not whole triangles");
        }
        double[] triangles = new double[order.length * 3];
        for (int i = 0; i < order.length; i++) {
            int at = order[i];
            if (at < 0 || at >= howMany) {
                throw new NotGltf("primitive of " + called + " indexes point " + at + " of " + howMany);
            }
            triangles[i * 3] = points[at * 3];
            triangles[i * 3 + 1] = points[at * 3 + 1];
            triangles[i * 3 + 2] = points[at * 3 + 2];
        }
        return new Part(name, path, triangles, colourOf(document.json, primitive));
    }

    private static String colourOf(JsonObject json, JsonObject primitive) {
        if (!primitive.has("material")) {
            return null;
        }
        int index = primitive.get("material").getAsInt();
        JsonArray materials = array(json, "materials");
        if (index < 0 || index >= materials.size()) {
            throw new NotGltf("a primitive names material " + index + ", which does not exist");
        }
        JsonObject material = materials.get(index).getAsJsonObject();
        if (!material.has("pbrMetallicRoughness")) {
            return null;
        }
        JsonObject pbr = material.getAsJsonObject("pbrMetallicRoughness");
        if (!pbr.has("baseColorFactor")) {
            return null;
        }
        JsonArray factor = pbr.getAsJsonArray("baseColorFactor");
        StringBuilder out = new StringBuilder("#");
        for (int i = 0; i < 3 && i < factor.size(); i++) {
            int channel = (int) Math.rint(factor.get(i).getAsDouble() * 255);
            out.append(String.format(Locale.ROOT, "%02x", Math.max(0, Math.min(255, channel))));
        }
        return out.toString();
    }

    private static double[] vectors(Document document, int index, int expected) {
        JsonArray accessors = array(document.json, "accessors");
        if (index < 0 || index >= accessors.size()) {
            throw new NotGltf("accessor " + index + " does not exist");
        }
        JsonObject accessor = accessors.get(index).getAsJsonObject();
        if (accessor.has("sparse")) {
            throw new NotGltf("accessor " + index + " is sparse, which this does not read");
        }
        if (!accessor.has("bufferView")) {
            throw new NotGltf("accessor " + index + " has no buffer view");
        }
        int componentType = accessor.get("componentType").getAsInt();
        if (!COMPONENT_WIDTHS.containsKey(componentType)) {
            throw new NotGltf(
                    "accessor " + index + " has component type " + componentType + ", which this does not read");
        }
        String type = accessor.has("type") ? accessor.get("type").getAsString() : null;
        if (!TYPE_COUNTS.containsKey(type)) {
            throw new NotGltf("accessor " + index + " has type " + type);
        }
        int width = COMPONENT_WIDTHS.get(componentType);
        int per = TYPE_COUNTS.get(type);
        int element = width * per;

        JsonArray views = array(document.json, "bufferViews");
        int which = accessor.get("bufferView").getAsInt();
        if (which < 0 || which >= views.size()) {
            throw new NotGltf("accessor " + index + " names a buffer view that does not exist");
        }
        JsonObject view = views.get(which).getAsJsonObject();
        int buffer = view.has("buffer") ? view.get("buffer").getAsInt() : 0;
        if (buffer < 0 || buffer >= document.buffers.size()) {
            throw new NotGltf("a buffer view names a buffer that does not exist");
        }
        byte[] data = document.buffers.get(buffer);
        int viewOffset = view.has("byteOffset") ? view.get("byteOffset").getAsInt() : 0;
        int stride = view.has("byteStride") && view.get("byteStride").getAsInt() > 0
                ? view.get("byteStride").getAsInt()
                : element;
        int start = viewOffset
                + (accessor.has("byteOffset") ? accessor.get("byteOffset").getAsInt() : 0);
        int count = accessor.get("count").getAsInt();

        long last = count == 0 ? start : (long) start + (long) stride * (count - 1) + element;
        if (count > 0
                && (start < 0 || last > viewOffset + view.get("byteLength").getAsLong() || last > data.length)) {
            throw new NotGltf("accessor " + index + " reads " + count
                    + " elements past the end of its buffer; the export is truncated or the accessor is wrong");
        }

        ByteBuffer bytes = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        double[] out = new double[count * per];
        for (int i = 0; i < count; i++) {
            int at = start + i * stride;
            for (int c = 0; c < per; c++) {
                out[i * per + c] = componentAt(bytes, componentType, at + c * width);
            }
        }
        return out;
    }

    private static double componentAt(ByteBuffer bytes, int componentType, int at) {
        switch (componentType) {
            case 5120:
                return bytes.get(at);
            case 5121:
                return bytes.get(at) & 0xFF;
            case 5122:
                return bytes.getShort(at);
            case 5123:
                return bytes.getShort(at) & 0xFFFF;
            case 5125:
                return bytes.getInt(at) & 0xFFFFFFFFL;
            case 5126:
                return bytes.getFloat(at);
            default:
                throw new NotGltf("component type " + componentType + " is not one this reads");
        }
    }

    private static Document parse(byte[] data) {
        if (data.length >= 4
                && data[0] == MAGIC[0]
                && data[1] == MAGIC[1]
                && data[2] == MAGIC[2]
                && data[3] == MAGIC[3]) {
            return unpackGlb(data);
        }
        JsonObject json;
        try {
            json = GSON.fromJson(new String(data, StandardCharsets.UTF_8), JsonObject.class);
        } catch (RuntimeException wrong) {
            throw new NotGltf("not a glb and not JSON; it begins "
                    + new String(Arrays.copyOf(data, Math.min(24, data.length)), StandardCharsets.UTF_8));
        }
        if (json == null || !json.has("asset")) {
            throw new NotGltf("JSON that is not a glTF document: no asset");
        }
        return new Document(json, buffersOf(json, new byte[0]));
    }

    private static Document unpackGlb(byte[] data) {
        if (data.length < 12) {
            throw new NotGltf("too short to be a glb: " + data.length + " bytes");
        }
        ByteBuffer bytes = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        int version = bytes.getInt(4);
        long length = bytes.getInt(8) & 0xFFFFFFFFL;
        if (version != VERSION) {
            throw new NotGltf("glb version " + version + "; this reads version " + VERSION);
        }
        if (length > data.length) {
            throw new NotGltf("glb says it is " + length + " bytes and " + data.length + " arrived");
        }

        JsonObject json = null;
        byte[] blob = new byte[0];
        int at = 12;
        while (at + 8 <= length) {
            int size = bytes.getInt(at);
            int kind = bytes.getInt(at + 4);
            at += 8;
            if (size < 0 || at + (long) size > length) {
                throw new NotGltf("a glb chunk runs past the end of the file");
            }
            byte[] chunk = Arrays.copyOfRange(data, at, at + size);
            if (kind == CHUNK_JSON && json == null) {
                json = GSON.fromJson(new String(chunk, StandardCharsets.UTF_8), JsonObject.class);
            } else if (kind == CHUNK_BIN && blob.length == 0) {
                blob = chunk;
            }
            at += size + padding(size);
        }
        if (json == null) {
            throw new NotGltf("the glb has no JSON chunk");
        }
        if (!json.has("asset")) {
            throw new NotGltf("JSON that is not a glTF document: no asset");
        }
        return new Document(json, buffersOf(json, blob));
    }

    private static List<byte[]> buffersOf(JsonObject json, byte[] blob) {
        List<byte[]> out = new ArrayList<>();
        JsonArray buffers = array(json, "buffers");
        for (int i = 0; i < buffers.size(); i++) {
            JsonObject buffer = buffers.get(i).getAsJsonObject();
            if (!buffer.has("uri")) {
                out.add(blob);
                continue;
            }
            String uri = buffer.get("uri").getAsString();
            if (!uri.startsWith("data:")) {
                throw new NotGltf("buffer " + i + " is a separate file ("
                        + uri.substring(0, Math.min(60, uri.length()))
                        + "); ask Onshape for a glb or for buffers inline");
            }
            int comma = uri.indexOf(',');
            try {
                out.add(Base64.getDecoder().decode(uri.substring(comma + 1)));
            } catch (IllegalArgumentException wrong) {
                throw new NotGltf("buffer " + i + " has a data uri that is not base64");
            }
        }
        return out;
    }

    static double[] identity() {
        return new double[] {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    }

    static double[] multiply(double[] a, double[] b) {
        double[] out = new double[16];
        for (int col = 0; col < 4; col++) {
            for (int row = 0; row < 4; row++) {
                double sum = 0;
                for (int k = 0; k < 4; k++) {
                    sum += a[k * 4 + row] * b[col * 4 + k];
                }
                out[col * 4 + row] = sum;
            }
        }
        return out;
    }

    static double[] placement(JsonObject node) {
        if (node.has("matrix")) {
            JsonArray matrix = node.getAsJsonArray("matrix");
            if (matrix.size() != 16) {
                throw new NotGltf("a node matrix has " + matrix.size() + " numbers rather than 16");
            }
            double[] out = new double[16];
            for (int i = 0; i < 16; i++) {
                out[i] = matrix.get(i).getAsDouble();
            }
            return out;
        }
        if (!node.has("translation") && !node.has("rotation") && !node.has("scale")) {
            return identity();
        }
        double[] translation = numbers(node, "translation", new double[] {0, 0, 0});
        double[] rotation = numbers(node, "rotation", new double[] {0, 0, 0, 1});
        double[] scale = numbers(node, "scale", new double[] {1, 1, 1});
        return fromTrs(translation, rotation, scale);
    }

    static double[] fromTrs(double[] translation, double[] rotation, double[] scale) {
        double x = rotation[0];
        double y = rotation[1];
        double z = rotation[2];
        double w = rotation[3];
        double[][] r = {
            {1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)},
            {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)},
            {2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)}
        };
        return new double[] {
            r[0][0] * scale[0],
            r[0][1] * scale[0],
            r[0][2] * scale[0],
            0,
            r[1][0] * scale[1],
            r[1][1] * scale[1],
            r[1][2] * scale[1],
            0,
            r[2][0] * scale[2],
            r[2][1] * scale[2],
            r[2][2] * scale[2],
            0,
            translation[0],
            translation[1],
            translation[2],
            1
        };
    }

    private static double[] numbers(JsonObject node, String name, double[] fallback) {
        if (!node.has(name)) {
            return fallback;
        }
        JsonArray given = node.getAsJsonArray(name);
        double[] out = new double[given.size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = given.get(i).getAsDouble();
        }
        return out;
    }

    private static JsonArray array(JsonObject json, String name) {
        return json.has(name) && json.get(name).isJsonArray() ? json.getAsJsonArray(name) : new JsonArray();
    }

    private static List<String> append(List<String> path, String name) {
        List<String> out = new ArrayList<>(path);
        out.add(name);
        return out;
    }

    private static int padding(int size) {
        return (4 - (size % 4)) % 4;
    }

    public static byte[] write(List<Part> parts) {
        return write(parts, "midnight-snackers TeamCode sim");
    }

    public static byte[] write(List<Part> parts, String generator) {
        JsonObject asset = new JsonObject();
        asset.addProperty("version", "2.0");
        asset.addProperty("generator", generator);

        JsonObject document = new JsonObject();
        document.add("asset", asset);
        document.addProperty("scene", 0);
        JsonArray scenes = new JsonArray();
        JsonObject scene = new JsonObject();
        JsonArray sceneNodes = new JsonArray();
        scene.add("nodes", sceneNodes);
        scenes.add(scene);
        document.add("scenes", scenes);
        JsonArray nodes = new JsonArray();
        JsonArray meshes = new JsonArray();
        JsonArray materials = new JsonArray();
        JsonArray accessors = new JsonArray();
        JsonArray views = new JsonArray();
        document.add("nodes", nodes);
        document.add("meshes", meshes);
        document.add("materials", materials);
        document.add("accessors", accessors);
        document.add("bufferViews", views);

        ByteArrayOutputStream blob = new ByteArrayOutputStream();
        Map<String, Integer> materialFor = new LinkedHashMap<>();
        Map<String, Integer> branches = new HashMap<>();

        for (Part part : parts) {
            if (part.count() == 0) {
                continue;
            }
            List<double[]> points = new ArrayList<>();
            int[] order = new int[part.count() * 3];
            Map<String, Integer> byPoint = new LinkedHashMap<>();
            for (int i = 0; i < order.length; i++) {
                double x = part.triangles[i * 3];
                double y = part.triangles[i * 3 + 1];
                double z = part.triangles[i * 3 + 2];
                String key = x + "," + y + "," + z;
                Integer at = byPoint.get(key);
                if (at == null) {
                    at = points.size();
                    byPoint.put(key, at);
                    points.add(new double[] {x, y, z});
                }
                order[i] = at;
            }

            ByteBuffer positions = ByteBuffer.allocate(points.size() * 12).order(ByteOrder.LITTLE_ENDIAN);
            double[] lows = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
            double[] highs = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
            for (double[] point : points) {
                for (int axis = 0; axis < 3; axis++) {
                    positions.putFloat((float) point[axis]);
                    lows[axis] = Math.min(lows[axis], point[axis]);
                    highs[axis] = Math.max(highs[axis], point[axis]);
                }
            }
            JsonObject position = new JsonObject();
            position.addProperty("bufferView", view(views, blob, positions.array(), ARRAY_BUFFER));
            position.addProperty("componentType", FLOAT);
            position.addProperty("count", points.size());
            position.addProperty("type", "VEC3");
            position.add("min", numbersOf(lows));
            position.add("max", numbersOf(highs));
            accessors.add(position);
            int positionIndex = accessors.size() - 1;

            boolean narrow = points.size() <= 0xFFFF;
            ByteBuffer indices =
                    ByteBuffer.allocate(order.length * (narrow ? 2 : 4)).order(ByteOrder.LITTLE_ENDIAN);
            for (int at : order) {
                if (narrow) {
                    indices.putShort((short) at);
                } else {
                    indices.putInt(at);
                }
            }
            JsonObject index = new JsonObject();
            index.addProperty("bufferView", view(views, blob, indices.array(), ELEMENT_ARRAY_BUFFER));
            index.addProperty("componentType", narrow ? UNSIGNED_SHORT : UNSIGNED_INT);
            index.addProperty("count", order.length);
            index.addProperty("type", "SCALAR");
            accessors.add(index);

            JsonObject attributes = new JsonObject();
            attributes.addProperty("POSITION", positionIndex);
            JsonObject primitive = new JsonObject();
            primitive.add("attributes", attributes);
            primitive.addProperty("indices", accessors.size() - 1);
            primitive.addProperty("mode", TRIANGLES);
            if (part.colour != null) {
                primitive.addProperty("material", materialIndex(materials, materialFor, part.colour));
            }
            JsonObject mesh = new JsonObject();
            mesh.addProperty("name", part.name);
            JsonArray primitives = new JsonArray();
            primitives.add(primitive);
            mesh.add("primitives", primitives);
            meshes.add(mesh);

            List<String> path = part.path.isEmpty() ? List.of(part.name) : part.path;
            JsonObject node = new JsonObject();
            node.addProperty("name", path.get(path.size() - 1));
            node.addProperty("mesh", meshes.size() - 1);
            nodes.add(node);
            int here = nodes.size() - 1;
            Integer parent =
                    path.size() > 1 ? branch(nodes, sceneNodes, branches, path.subList(0, path.size() - 1)) : null;
            if (parent == null) {
                sceneNodes.add(here);
            } else {
                childrenOf(nodes.get(parent).getAsJsonObject()).add(here);
            }
        }

        if (materials.size() == 0) {
            document.remove("materials");
        }
        JsonArray buffers = new JsonArray();
        byte[] bytes = blob.toByteArray();
        if (bytes.length > 0) {
            JsonObject buffer = new JsonObject();
            buffer.addProperty("byteLength", bytes.length);
            buffers.add(buffer);
        }
        document.add("buffers", buffers);

        byte[] text = GSON.toJson(document).getBytes(StandardCharsets.UTF_8);
        byte[] json = Arrays.copyOf(text, text.length + padding(text.length));
        Arrays.fill(json, text.length, json.length, (byte) ' ');
        byte[] binary = Arrays.copyOf(bytes, bytes.length + padding(bytes.length));

        int total = 12 + 8 + json.length + (binary.length > 0 ? 8 + binary.length : 0);
        ByteBuffer out = ByteBuffer.allocate(total).order(ByteOrder.LITTLE_ENDIAN);
        out.put(MAGIC);
        out.putInt(VERSION);
        out.putInt(total);
        out.putInt(json.length);
        out.putInt(CHUNK_JSON);
        out.put(json);
        if (binary.length > 0) {
            out.putInt(binary.length);
            out.putInt(CHUNK_BIN);
            out.put(binary);
        }
        return out.array();
    }

    private static JsonArray numbersOf(double[] values) {
        JsonArray out = new JsonArray();
        for (double value : values) {
            out.add(value);
        }
        return out;
    }

    private static JsonArray childrenOf(JsonObject node) {
        if (!node.has("children")) {
            node.add("children", new JsonArray());
        }
        return node.getAsJsonArray("children");
    }

    private static int view(JsonArray views, ByteArrayOutputStream blob, byte[] data, int target) {
        int pad = padding(blob.size());
        for (int i = 0; i < pad; i++) {
            blob.write(0);
        }
        int offset = blob.size();
        blob.write(data, 0, data.length);
        JsonObject view = new JsonObject();
        view.addProperty("buffer", 0);
        view.addProperty("byteOffset", offset);
        view.addProperty("byteLength", data.length);
        view.addProperty("target", target);
        views.add(view);
        return views.size() - 1;
    }

    private static int materialIndex(JsonArray materials, Map<String, Integer> known, String colour) {
        Integer had = known.get(colour);
        if (had != null) {
            return had;
        }
        JsonArray factor = new JsonArray();
        for (int i = 1; i < 7; i += 2) {
            factor.add(Integer.parseInt(colour.substring(i, i + 2), 16) / 255.0);
        }
        factor.add(1.0);
        JsonObject pbr = new JsonObject();
        pbr.add("baseColorFactor", factor);
        pbr.addProperty("metallicFactor", 0.0);
        pbr.addProperty("roughnessFactor", 0.7);
        JsonObject material = new JsonObject();
        material.addProperty("name", colour);
        material.add("pbrMetallicRoughness", pbr);
        materials.add(material);
        known.put(colour, materials.size() - 1);
        return materials.size() - 1;
    }

    private static Integer branch(
            JsonArray nodes, JsonArray sceneNodes, Map<String, Integer> branches, List<String> path) {
        Integer parent = null;
        for (int depth = 0; depth < path.size(); depth++) {
            String key = String.join("\u0000", path.subList(0, depth + 1));
            Integer at = branches.get(key);
            if (at == null) {
                JsonObject node = new JsonObject();
                node.addProperty("name", path.get(depth));
                nodes.add(node);
                at = nodes.size() - 1;
                branches.put(key, at);
                if (parent == null) {
                    sceneNodes.add(at);
                } else {
                    childrenOf(nodes.get(parent).getAsJsonObject()).add(at);
                }
            }
            parent = at;
        }
        return parent;
    }
}
