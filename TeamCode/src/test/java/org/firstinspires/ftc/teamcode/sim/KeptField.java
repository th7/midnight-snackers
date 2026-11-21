package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HexFormat;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.DrawnField;
import org.firstinspires.ftc.teamcode.simcore.FieldModel;
import org.firstinspires.ftc.teamcode.simcore.FieldOffer;
import org.firstinspires.ftc.teamcode.simcore.WhereTheHostIs;

public final class KeptField {
    static final int CONNECT_MILLIS = 2_000;
    static final int READ_MILLIS = 30_000;

    private static final Gson GSON = new Gson();

    private final Path dir;
    private final Store store;

    public KeptField(Path dir, Store store) {
        this.dir = dir;
        this.store = store;
    }

    public Path dir() {
        return dir;
    }

    public List<String> fetchFrom(WhereTheHostIs where) {
        if (where instanceof WhereTheHostIs.At at) {
            return fetchFrom(at);
        }
        return List.of("  field  " + WhereTheHostIs.ENV + " is not set, so the field models are not fetched from the"
                + " host; drawing " + drawn().said());
    }

    private List<String> fetchFrom(WhereTheHostIs.At at) {
        List<String> said = new ArrayList<>();
        Checked<FieldOffer> offer;
        try {
            offer = offerFrom(at);
        } catch (IOException unreached) {
            said.add("  field  could not reach the host at " + at + " (" + unreached.getMessage() + ")");
            said.add("  field  drawing " + drawn().said());
            return said;
        }
        said.addAll(offer.fold(
                offered -> fetchedAll(at, offered),
                refused -> List.of("  field  the host at " + at + " offers nothing this simulator keeps: " + refused)));
        said.add("  field  drawing " + drawn().said());
        return said;
    }

    private List<String> fetchedAll(WhereTheHostIs.At at, FieldOffer offered) {
        List<FieldOffer.Offered> fetching = offered.toFetch(keptSha256());
        if (fetching.isEmpty()) {
            return List.of("  field  every model the host at " + at + " offers is already kept in " + dir);
        }
        List<String> said = new ArrayList<>();
        for (FieldOffer.Offered one : fetching) {
            said.add("  field  " + fetched(at, one));
        }
        return said;
    }

    private Checked<FieldOffer> offerFrom(WhereTheHostIs.At at) throws IOException {
        HttpURLConnection connection = opened(at.url() + "field");
        try {
            int status = connection.getResponseCode();
            if (status != 200) {
                return Checked.rejected("it answered " + status + ": " + text(connection.getErrorStream()));
            }
            String body = text(connection.getInputStream());
            try {
                JsonObject offer = GSON.fromJson(body, JsonObject.class);
                List<FieldOffer.Said> models = new ArrayList<>();
                for (JsonElement element : offer.getAsJsonArray("models")) {
                    JsonObject model = element.getAsJsonObject();
                    models.add(new FieldOffer.Said(
                            model.get("resolution").getAsString(),
                            model.get("file").getAsString(),
                            model.get("bytes").getAsLong(),
                            model.get("sha256").getAsString()));
                }
                return FieldOffer.read(offer.get("version").getAsInt(), models);
            } catch (JsonParseException
                    | IllegalStateException
                    | NullPointerException
                    | UnsupportedOperationException
                    | NumberFormatException wrong) {
                return Checked.rejected("its offer is not one: " + body);
            }
        } finally {
            connection.disconnect();
        }
    }

    private String fetched(WhereTheHostIs.At at, FieldOffer.Offered one) {
        String file = one.model().file();
        byte[] body;
        try {
            HttpURLConnection connection = opened(at.url() + "field/" + file);
            try {
                int status = connection.getResponseCode();
                if (status != 200) {
                    return file + " was not fetched: the host answered " + status + ": "
                            + text(connection.getErrorStream());
                }
                try (InputStream in = connection.getInputStream()) {
                    body = in.readAllBytes();
                }
            } finally {
                connection.disconnect();
            }
        } catch (IOException cut) {
            return file + " was cut off (" + cut.getMessage() + "); the one kept before is kept";
        }
        if (body.length != one.bytes() || !sha256(body).equals(one.sha256())) {
            return file + " arrived as " + body.length + " bytes that are not the " + one.bytes()
                    + " the host offered; the one kept before is kept";
        }
        store.writeWhole(dir.resolve(file), body);
        return "fetched the " + one.model().asked() + " model from the host, " + body.length + " bytes, into " + dir;
    }

    public DrawnField drawn() {
        return DrawnField.of(kept());
    }

    public Response serve(String name) {
        if (FieldAssets.DEFAULT_RESOLUTION_FILE.equals(name)) {
            Set<FieldModel> kept = kept();
            return Response.bytes(
                            "text/javascript; charset=utf-8",
                            FieldAssets.defaultResolutionModule(
                                            DrawnField.of(kept).model(), DrawnField.drawable(kept))
                                    .getBytes(StandardCharsets.UTF_8))
                    .revalidated();
        }
        for (FieldModel one : kept()) {
            if (one.file().equals(name)) {
                return SimAssets.serveUnder(dir, store, name);
            }
        }
        return SimAssets.serve(name);
    }

    private Set<FieldModel> kept() {
        Set<FieldModel> kept = EnumSet.noneOf(FieldModel.class);
        for (FieldModel one : FieldModel.values()) {
            if (store.isFile(dir.resolve(one.file()))) {
                kept.add(one);
            }
        }
        return kept;
    }

    private Map<FieldModel, String> keptSha256() {
        Map<FieldModel, String> digests = new EnumMap<>(FieldModel.class);
        for (FieldModel one : kept()) {
            store.readIfThere(dir.resolve(one.file())).ifPresent(bytes -> digests.put(one, sha256(bytes)));
        }
        return digests;
    }

    private static HttpURLConnection opened(String url) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(url).openConnection();
        connection.setConnectTimeout(CONNECT_MILLIS);
        connection.setReadTimeout(READ_MILLIS);
        connection.setUseCaches(false);
        return connection;
    }

    private static String text(InputStream in) throws IOException {
        if (in == null) {
            return "";
        }
        try (in) {
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        }
    }

    private static String sha256(byte[] bytes) {
        try {
            return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(bytes));
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException("every JVM has SHA-256", e);
        }
    }
}
