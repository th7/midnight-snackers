package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.Test;

public class OnshapeTest {
    private static final String ACCESS = "AAAAAAAAAAAAAAAAAAAAAAAA";
    private static final String SECRET = "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB";
    private static final String NONCE = "AbCdEfGhIjKlMnOpQrStUvWxY";
    private static final String DATE = "Mon, 18 Sep 2026 12:00:00 GMT";
    private static final String PATH = "/api/v10/documents/abc";
    private static final String JSON = "application/json";

    private static String sign() {
        return Onshape.signature("GET", PATH, "", NONCE, DATE, JSON, SECRET);
    }

    @Test
    public void aKnownCallSignsToAKnownString() {
        assertEquals("RAElJvpy1mwpaT9YvdNVKGITbAapDFv0aOfvrF6R7XM=", sign());
    }

    @Test
    public void everyPartOfTheCallIsSigned() {
        assertNotEquals("the method", sign(), Onshape.signature("POST", PATH, "", NONCE, DATE, JSON, SECRET));
        assertNotEquals(
                "the path", sign(), Onshape.signature("GET", "/api/v10/documents/xyz", "", NONCE, DATE, JSON, SECRET));
        assertNotEquals("the query", sign(), Onshape.signature("GET", PATH, "foo=bar", NONCE, DATE, JSON, SECRET));
        assertNotEquals(
                "the nonce",
                sign(),
                Onshape.signature("GET", PATH, "", "ZzZzZzZzZzZzZzZzZzZzZzZzZ", DATE, JSON, SECRET));
        assertNotEquals(
                "the date",
                sign(),
                Onshape.signature("GET", PATH, "", NONCE, "Tue, 19 Sep 2026 12:00:00 GMT", JSON, SECRET));
        assertNotEquals(
                "the content type", sign(), Onshape.signature("GET", PATH, "", NONCE, DATE, "text/plain", SECRET));
        assertNotEquals("the secret", sign(), Onshape.signature("GET", PATH, "", NONCE, DATE, JSON, "C".repeat(44)));
    }

    @Test
    public void theCaseOfTheCallDoesNotChangeTheSignature() {
        assertEquals(sign(), Onshape.signature("get", PATH, "", NONCE, DATE, JSON, SECRET));
        assertEquals(sign(), Onshape.signature("GET", "/API/V10/Documents/ABC", "", NONCE, DATE, JSON, SECRET));
    }

    @Test
    public void theAccessKeyIsNotSignedButNamesTheSigner() {
        String header = Onshape.authorization(ACCESS, sign());

        assertTrue(header, header.startsWith("On " + ACCESS + ":HmacSHA256:"));
        assertTrue(header, header.contains(sign()));
    }

    @Test
    public void aSignedRequestCarriesWhatTheSignatureCovers() {
        Map<String, String> headers = Onshape.signedHeaders("GET", PATH, "", ACCESS, SECRET, NONCE, DATE, JSON);

        assertEquals(NONCE, headers.get("On-Nonce"));
        assertEquals(DATE, headers.get("Date"));
        assertTrue(headers.get("Authorization").contains(sign()));
    }

    @Test
    public void eachRequestGetsAFreshNonce() {
        Set<String> nonces = new HashSet<>();
        for (int i = 0; i < 50; i++) {
            nonces.add(Onshape.nonce());
        }

        assertEquals(50, nonces.size());
    }

    @Test
    public void theNonceIsLongEnoughToBeUnguessable() {
        String nonce = Onshape.nonce();

        assertTrue(nonce, nonce.length() >= 25);
        assertTrue(nonce, nonce.matches("[A-Za-z0-9]+"));
    }

    @Test
    public void keysWithoutEitherVariableAreRefusedNamingBoth() {
        Onshape.NoCredentials refused = assertThrows(Onshape.NoCredentials.class, () -> Onshape.keysIn(Map.of()));

        assertTrue(refused.getMessage(), refused.getMessage().contains(Onshape.ACCESS_KEY_VARIABLE));
        assertTrue(refused.getMessage(), refused.getMessage().contains(Onshape.SECRET_KEY_VARIABLE));
    }

    @Test
    public void halfAKeyPairIsStillMissing() {
        Onshape.NoCredentials refused = assertThrows(
                Onshape.NoCredentials.class, () -> Onshape.keysIn(Map.of(Onshape.ACCESS_KEY_VARIABLE, ACCESS)));

        assertTrue(refused.getMessage(), refused.getMessage().contains(Onshape.SECRET_KEY_VARIABLE));
    }

    @Test
    public void aBlankKeyIsMissingRatherThanAKey() {
        assertThrows(
                Onshape.NoCredentials.class,
                () -> Onshape.keysIn(Map.of(Onshape.ACCESS_KEY_VARIABLE, ACCESS, Onshape.SECRET_KEY_VARIABLE, "   ")));
    }

    @Test
    public void neitherKeyIsInTheMessageThatSaysOneIsMissing() {
        Onshape.NoCredentials refused = assertThrows(
                Onshape.NoCredentials.class,
                () -> Onshape.keysIn(Map.of(Onshape.ACCESS_KEY_VARIABLE, ACCESS, Onshape.SECRET_KEY_VARIABLE, "")));

        assertFalse(refused.getMessage(), refused.getMessage().contains(ACCESS));
    }

    @Test
    public void anUnsignedRequestSetsNoneOfTheThreeHeadersTheProxyOwns() {
        Map<String, String> sent = Onshape.unsignedHeaders(JSON);

        for (String header : List.of("Authorization", "Date", "On-Nonce")) {
            assertFalse(header + " is the proxy's to set", sent.containsKey(header));
        }
    }

    @Test
    public void aClientWithoutKeysIsBuiltRatherThanRefused() {
        assertFalse(Onshape.configured(Map.of(), refusing(401)).signed());
    }

    @Test
    public void aClientSignsForItselfWhenTheKeysAreInTheEnvironment() {
        Onshape signing = Onshape.configured(
                Map.of(Onshape.ACCESS_KEY_VARIABLE, ACCESS, Onshape.SECRET_KEY_VARIABLE, SECRET), refusing(401));

        assertTrue(signing.signed());
    }

    @Test
    public void aRefusalOfAnUnsignedRequestNamesBothWaysToBeAuthenticated() {
        Onshape anonymous = Onshape.configured(Map.of(), refusing(401));

        Onshape.NoCredentials refused =
                assertThrows(Onshape.NoCredentials.class, () -> anonymous.get("/api/v10/assemblies/d/a/w/b/e/c/gltf"));

        assertTrue(refused.getMessage(), refused.getMessage().contains(Onshape.ACCESS_KEY_VARIABLE));
        assertTrue(refused.getMessage(), refused.getMessage().contains("proxy"));
    }

    @Test
    public void aRefusalOfASignedRequestSaysTheKeysWereRefusedAndDoesNotRepeatThem() {
        Onshape signing = Onshape.configured(
                Map.of(Onshape.ACCESS_KEY_VARIABLE, ACCESS, Onshape.SECRET_KEY_VARIABLE, SECRET), refusing(403));

        Onshape.NoCredentials refused =
                assertThrows(Onshape.NoCredentials.class, () -> signing.get("/api/v10/assemblies/d/a/w/b/e/c/gltf"));

        assertTrue(refused.getMessage(), refused.getMessage().toLowerCase().contains("refused"));
        assertFalse(refused.getMessage(), refused.getMessage().contains(SECRET));
    }

    @Test
    public void aFailureThatIsNotAboutCredentialsIsNotReportedAsOne() {
        Onshape anonymous = Onshape.configured(Map.of(), refusing(500));

        RuntimeException thrown =
                assertThrows(RuntimeException.class, () -> anonymous.get("/api/v10/assemblies/d/a/w/b/e/c/gltf"));

        assertFalse(thrown.getClass().getName(), thrown instanceof Onshape.NoCredentials);
        assertTrue(thrown.getMessage(), thrown.getMessage().contains("500"));
    }

    @Test
    public void aClientDoesNotPrintItsKeys() {
        Onshape signing = Onshape.configured(
                Map.of(Onshape.ACCESS_KEY_VARIABLE, ACCESS, Onshape.SECRET_KEY_VARIABLE, SECRET), refusing(401));

        assertFalse(signing.toString(), signing.toString().contains(ACCESS));
        assertFalse(signing.toString(), signing.toString().contains(SECRET));
    }

    @Test
    public void aPathAndItsQueryAreKeptApart() {
        assertEquals(
                "/api/v10/documents/d/a/w/b/elements",
                Onshape.pathOf("/api/v10/documents/d/a/w/b/elements?elementType=BLOB"));
        assertEquals("elementType=BLOB", Onshape.queryOf("/api/v10/documents/d/a/w/b/elements?elementType=BLOB"));
        assertEquals("/api/v10/documents/abc", Onshape.pathOf("/api/v10/documents/abc"));
        assertEquals("", Onshape.queryOf("/api/v10/documents/abc"));
    }

    @Test
    public void whatWasSentIsWhatWasSigned() {
        Map<String, String>[] seen = asked();
        Onshape signing = Onshape.configured(
                Map.of(Onshape.ACCESS_KEY_VARIABLE, ACCESS, Onshape.SECRET_KEY_VARIABLE, SECRET), (url, headers) -> {
                    seen[0] = headers;
                    return new Onshape.Answer(200, new byte[] {1, 2, 3});
                });

        byte[] body = signing.get("/api/v10/documents/abc");

        assertEquals(3, body.length);
        assertEquals(
                Onshape.authorization(
                        ACCESS,
                        Onshape.signature("GET", PATH, "", seen[0].get("On-Nonce"), seen[0].get("Date"), JSON, SECRET)),
                seen[0].get("Authorization"));
    }

    @SuppressWarnings("unchecked")
    private static Map<String, String>[] asked() {
        return new Map[1];
    }

    private static Onshape.Calls refusing(int status) {
        return (url, headers) ->
                new Onshape.Answer(status, "{\"message\": \"Unauthenticated API request\"}".getBytes());
    }
}
