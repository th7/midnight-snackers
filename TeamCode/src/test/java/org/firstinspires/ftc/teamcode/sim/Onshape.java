package org.firstinspires.ftc.teamcode.sim;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.security.SecureRandom;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Base64;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import javax.crypto.Mac;
import javax.crypto.spec.SecretKeySpec;

public final class Onshape {
    public static final String BASE = "https://cad.onshape.com";
    public static final String FIELD_DOCUMENT = "a355e772e3d24813de7852ee";
    public static final String FIELD_WORKSPACE = "f106353168f1f92100b81259";
    public static final String FIELD_ASSEMBLY = "95d1e1e442b4138cccaf2d73";
    public static final String ACCESS_KEY_VARIABLE = "ONSHAPE_ACCESS_KEY";
    public static final String SECRET_KEY_VARIABLE = "ONSHAPE_SECRET_KEY";
    public static final String JSON = "application/json";

    private static final int NONCE_LENGTH = 25;
    private static final String ALPHANUMERIC = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    private static final SecureRandom RANDOM = new SecureRandom();
    private static final DateTimeFormatter RFC_1123 =
            DateTimeFormatter.ofPattern("EEE, dd MMM yyyy HH:mm:ss 'GMT'", Locale.US);
    private static final int BODY_IN_A_REFUSAL = 400;

    public static final class NoCredentials extends RuntimeException {
        public NoCredentials(String message) {
            super(message);
        }
    }

    public static final class Answer {
        public final int status;
        public final byte[] body;

        public Answer(int status, byte[] body) {
            this.status = status;
            this.body = body;
        }
    }

    public interface Calls {
        Answer get(String url, Map<String, String> headers);
    }

    public static final class Keys {
        final String access;
        final String secret;

        Keys(String access, String secret) {
            this.access = access;
            this.secret = secret;
        }
    }

    private final Keys keys;
    private final Calls calls;
    private final String base;

    private Onshape(Keys keys, Calls calls, String base) {
        this.keys = keys;
        this.calls = calls;
        this.base = base;
    }

    public static Onshape configured(Map<String, String> environment, Calls calls) {
        return new Onshape(keysOrNoneIn(environment), calls, BASE);
    }

    public static Onshape configured(Map<String, String> environment) {
        return configured(environment, overTheNetwork());
    }

    public static Onshape configured() {
        return configured(System.getenv());
    }

    public static Onshape authenticated(Map<String, String> environment, Calls calls) {
        return new Onshape(keysIn(environment), calls, BASE);
    }

    public boolean signed() {
        return keys != null;
    }

    @Override
    public String toString() {
        return "<Onshape " + base + (signed() ? " signed>" : " anonymous>");
    }

    public byte[] get(String url) {
        return get(url, JSON);
    }

    public byte[] get(String url, String accept) {
        String path = pathOf(url);
        Map<String, String> headers = signed()
                ? signedHeaders("GET", path, queryOf(url), keys.access, keys.secret, nonce(), now(), JSON)
                : unsignedHeaders(JSON);
        headers.put("Accept", accept);

        Answer answer = calls.get(base + url, headers);
        if (answer.status == 401 || answer.status == 403) {
            throw new NoCredentials(refusal(path, answer.status, said(answer)));
        }
        if (answer.status < 200 || answer.status >= 300) {
            throw new IllegalStateException("Onshape " + answer.status + " for " + path + ": " + said(answer));
        }
        return answer.body;
    }

    private static String said(Answer answer) {
        String body = new String(answer.body == null ? new byte[0] : answer.body, StandardCharsets.UTF_8);
        return body.length() > BODY_IN_A_REFUSAL ? body.substring(0, BODY_IN_A_REFUSAL) : body;
    }

    private String refusal(String path, int status, String body) {
        if (signed()) {
            return "Onshape refused the signed request for " + path + " (" + status
                    + "): the key pair was refused -- revoked, deleted, or scoped away from this document. "
                    + "Make a new one and set " + ACCESS_KEY_VARIABLE + " and " + SECRET_KEY_VARIABLE + ". " + body;
        }
        return "Onshape refused an unsigned request for " + path + " (" + status
                + ") and nothing authenticated it. Either an egress proxy is meant to be signing "
                + "cad.onshape.com on the way out and its onshape service is not switched on, or there is no "
                + "proxy here and this machine needs its own key pair in " + ACCESS_KEY_VARIABLE + " and "
                + SECRET_KEY_VARIABLE + ". " + body;
    }

    public static Keys keysIn(Map<String, String> environment) {
        String access = trimmed(environment.get(ACCESS_KEY_VARIABLE));
        String secret = trimmed(environment.get(SECRET_KEY_VARIABLE));
        List<String> missing = new ArrayList<>();
        if (access.isEmpty()) {
            missing.add(ACCESS_KEY_VARIABLE);
        }
        if (secret.isEmpty()) {
            missing.add(SECRET_KEY_VARIABLE);
        }
        if (!missing.isEmpty()) {
            throw new NoCredentials("Onshape geometry needs a key pair and " + String.join(" and ", missing)
                    + (missing.size() > 1 ? " are not set. " : " is not set. ")
                    + "Make one at dev-portal.onshape.com and export it; never commit it. "
                    + "(The public parts of the document -- metadata, BOM, blobs -- need no keys.)");
        }
        return new Keys(access, secret);
    }

    private static Keys keysOrNoneIn(Map<String, String> environment) {
        try {
            return keysIn(environment);
        } catch (NoCredentials none) {
            return null;
        }
    }

    private static String trimmed(String value) {
        return value == null ? "" : value.trim();
    }

    public static String pathOf(String url) {
        int mark = url.indexOf('?');
        return mark < 0 ? url : url.substring(0, mark);
    }

    public static String queryOf(String url) {
        int mark = url.indexOf('?');
        return mark < 0 ? "" : url.substring(mark + 1);
    }

    public static String nonce() {
        StringBuilder out = new StringBuilder(NONCE_LENGTH);
        for (int i = 0; i < NONCE_LENGTH; i++) {
            out.append(ALPHANUMERIC.charAt(RANDOM.nextInt(ALPHANUMERIC.length())));
        }
        return out.toString();
    }

    public static String now() {
        return ZonedDateTime.now(ZoneOffset.UTC).format(RFC_1123);
    }

    public static String signature(
            String method, String path, String query, String nonce, String date, String contentType, String secret) {
        String signed = String.join("\n", method, nonce, date, contentType, path, query, "")
                .toLowerCase(Locale.ROOT);
        try {
            Mac mac = Mac.getInstance("HmacSHA256");
            mac.init(new SecretKeySpec(secret.getBytes(StandardCharsets.UTF_8), "HmacSHA256"));
            return Base64.getEncoder().encodeToString(mac.doFinal(signed.getBytes(StandardCharsets.UTF_8)));
        } catch (java.security.GeneralSecurityException e) {
            throw new IllegalStateException("this JVM cannot sign for Onshape", e);
        }
    }

    public static String authorization(String access, String signature) {
        return "On " + access + ":HmacSHA256:" + signature;
    }

    public static Map<String, String> unsignedHeaders(String contentType) {
        Map<String, String> sent = new LinkedHashMap<>();
        sent.put("Accept", JSON);
        sent.put("Content-Type", contentType);
        return sent;
    }

    public static Map<String, String> signedHeaders(
            String method,
            String path,
            String query,
            String access,
            String secret,
            String nonce,
            String date,
            String contentType) {
        Map<String, String> sent = unsignedHeaders(contentType);
        sent.put("On-Nonce", nonce);
        sent.put("Date", date);
        sent.put(
                "Authorization",
                authorization(access, signature(method, path, query, nonce, date, contentType, secret)));
        return sent;
    }

    public static Calls overTheNetwork() {
        return (url, headers) -> {
            HttpURLConnection connection = null;
            try {
                connection = (HttpURLConnection) new URL(url).openConnection();
                connection.setRequestMethod("GET");
                connection.setInstanceFollowRedirects(true);
                for (Map.Entry<String, String> header : headers.entrySet()) {
                    connection.setRequestProperty(header.getKey(), header.getValue());
                }
                int status = connection.getResponseCode();
                InputStream in = status >= 400 ? connection.getErrorStream() : connection.getInputStream();
                return new Answer(status, in == null ? new byte[0] : readWhole(in));
            } catch (IOException e) {
                throw new UncheckedIOException("could not reach Onshape at " + pathOf(url), e);
            } finally {
                if (connection != null) {
                    connection.disconnect();
                }
            }
        };
    }

    private static byte[] readWhole(InputStream in) throws IOException {
        try (InputStream open = in) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] chunk = new byte[16384];
            int read;
            while ((read = open.read(chunk)) > 0) {
                out.write(chunk, 0, read);
            }
            return out.toByteArray();
        }
    }
}
