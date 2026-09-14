package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Map;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.junit.Test;

/**
 * A router is the one place a server says which method and path reach which handler, so 404,
 * 405, path parameters, a guard over a whole family of routes, and mounting one router's routes
 * under another's prefix are each decided once.
 */
public class RouterTest {
    private static Request get(String target) {
        return Request.of("GET", target, "");
    }

    private static Request post(String target) {
        return Request.of("POST", target, "");
    }

    private static Router.Handler say(String what) {
        return (request, params) -> Response.json(what);
    }

    @Test
    public void routesByMethodAndPathAndSaysNotFoundOtherwise() {
        Router router = new Router().route("GET", "/status", say("status")).route("POST", "/run", say("ran"));

        assertEquals("status", router.handle(get("/status")).body);
        assertEquals("ran", router.handle(post("/run")).body);
        Response missing = router.handle(get("/nope"));
        assertEquals(404, missing.status);
        assertEquals("not found: /nope", missing.body);
    }

    @Test
    public void aKnownPathWithTheWrongMethodIs405NamingWhatIsAllowed() {
        Router router =
                new Router().route("GET", "/files/{key*}", say("read")).route("PUT", "/files/{key*}", say("written"));

        Response refused = router.handle(post("/files/a/b.java"));

        assertEquals(405, refused.status);
        assertEquals("GET or PUT /files/a/b.java", refused.body);
    }

    @Test
    public void aSegmentParameterTakesOneSegmentAndAStarredOneTheRestOfThePath() {
        Router router = new Router()
                .route("GET", "/runs/{id}/ticks", (request, params) -> Response.json("ticks of " + params.get("id")))
                .route("GET", "/runs/{id}/", (request, params) -> Response.json("page of " + params.get("id")))
                .route("GET", "/files/{key*}", (request, params) -> Response.json("file " + params.get("key")));

        assertEquals("ticks of 3", router.handle(get("/runs/3/ticks?from=2")).body);
        assertEquals("page of 3", router.handle(get("/runs/3/")).body);
        assertEquals("file TeamCode/src/Plans.java", router.handle(get("/files/TeamCode/src/Plans.java")).body);
        assertEquals("a segment parameter is one segment", 404, router.handle(get("/runs/3/4/ticks")).status);
        assertEquals("the trailing slash is part of the path", 404, router.handle(get("/runs/3")).status);
    }

    @Test
    public void aMountedRouterAnswersUnderItsPrefixWithThePrefixStripped() {
        Router bench = new Router().route("GET", "/status", say("bench status"));
        Router server = new Router().route("GET", "/", say("page")).mount("/sim", bench);

        assertEquals("bench status", server.handle(get("/sim/status")).body);
        assertEquals("page", server.handle(get("/")).body);
        assertEquals(404, server.handle(get("/status")).status);
        assertEquals(404, server.handle(get("/simstatus")).status);
        assertEquals(
                "the mount's own not-found names the full path",
                "not found: /sim/nope",
                server.handle(get("/sim/nope")).body);
    }

    @Test
    public void aMountAtTheRootAndAMountChosenPerRequest() {
        Router ada = new Router().route("GET", "/status", say("ada's"));
        Router bob = new Router().route("GET", "/status", say("bob's"));
        Router server = new Router()
                .mount("", new Router().route("GET", "/", say("page")))
                .mount("/sim", request -> "ada".equals(request.header("x-user")) ? ada : bob);

        assertEquals("page", server.handle(get("/")).body);
        assertEquals("ada's", server.handle(Request.of("GET", "/sim/status", Map.of("x-user", "ada"), "")).body);
        assertEquals("bob's", server.handle(get("/sim/status")).body);
    }

    @Test
    public void aGuardRefusesEveryRouteAndMountItCoversButNotWhatIsNotThere() {
        Router guarded = new Router()
                .guard(request ->
                        "yes".equals(request.header("x-approved")) ? null : Response.error(403, "not approved"))
                .route("GET", "/files", say("files"))
                .route("POST", "/git/push", say("pushed"))
                .mount("/sim", new Router().route("GET", "/status", say("status")));
        Router server = new Router().route("GET", "/", say("login")).mount("", guarded);

        assertEquals("login", server.handle(get("/")).body);
        assertEquals(403, server.handle(get("/files")).status);
        assertEquals(403, server.handle(get("/sim/status")).status);
        assertEquals("the guard comes before the method check", 403, server.handle(get("/git/push")).status);
        assertEquals("an unknown path is unknown to everyone", 404, server.handle(get("/nope")).status);
        Request approved = Request.of("GET", "/files", Map.of("x-approved", "yes"), "");
        assertEquals("files", server.handle(approved).body);
        assertEquals("status", server.handle(Request.of("GET", "/sim/status", Map.of("x-approved", "yes"), "")).body);
    }

    /** Choosing the mounted router may need what the guard checks, e.g. an approved user's own bench. */
    @Test
    public void theGuardAnswersForEverythingUnderAMountsPrefixBeforeTheMountedRouterIsChosen() {
        Router server = new Router()
                .guard(request ->
                        "yes".equals(request.header("x-approved")) ? null : Response.error(403, "not approved"))
                .mount("/sim", request -> {
                    throw new IllegalStateException("chose a router for a request the guard should have refused");
                });

        assertEquals(403, server.handle(get("/sim/status")).status);
        assertEquals("even a path the mounted router would not know", 403, server.handle(get("/sim/nope")).status);
        assertEquals(404, server.handle(get("/simulator")).status);
    }

    @Test
    public void aRequestMadeByHandParsesItsQueryLikeOneOffTheWire() {
        Request request = Request.of("GET", "/runs/1/ticks?from=7&name=Count%20to%20three", "");

        assertEquals("/runs/1/ticks", request.path);
        assertEquals("7", request.query("from"));
        assertEquals(7, request.queryInt("from", 0));
        assertEquals(0, request.queryInt("missing", 0));
        assertEquals("Count to three", request.query("name"));
        assertTrue(request.remoteAddress.isLoopbackAddress());
    }
}
