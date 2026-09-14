package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

/**
 * Which method and path reach which handler, said once per server. A route's pattern is a path
 * whose {@code {name}} segments are parameters and whose final {@code {name*}} takes the rest of
 * the path, slashes included. Routes are tried in the order they were added. A path no route or
 * mount knows is a 404; a path a route knows by another method is a 405 naming the methods that
 * would do. A {@link #guard} answers for every route here and every path under a mount's
 * prefix before any of them does, but never for a path none of them knows.
 * <p>
 * {@link #mount} puts another router's routes under a prefix, so the same routes can be served
 * at the root of one server and under {@code /sim} on another; the mounted router sees the path
 * with the prefix taken off, and its own not-found names the full path.
 */
public final class Router implements Function<Request, Response> {
    public interface Handler {
        /**
         * @param params the pattern's parameters, by name
         */
        Response handle(Request request, Map<String, String> params);
    }

    private static final class Route {
        final String method;
        final String[] segments;
        final boolean rest;
        final Handler handler;

        Route(String method, String pattern, Handler handler) {
            this.method = method;
            this.segments = pattern.split("/", -1);
            String last = segments[segments.length - 1];
            this.rest = last.startsWith("{") && last.endsWith("*}");
            this.handler = handler;
        }

        /** The parameters when {@code path} matches, else null. */
        Map<String, String> match(String path) {
            String[] parts = path.split("/", -1);
            if (rest ? parts.length < segments.length : parts.length != segments.length) {
                return null;
            }
            Map<String, String> params = new LinkedHashMap<>();
            for (int i = 0; i < segments.length; i++) {
                String segment = segments[i];
                if (rest && i == segments.length - 1) {
                    params.put(
                            segment.substring(1, segment.length() - 2),
                            String.join("/", java.util.Arrays.copyOfRange(parts, i, parts.length)));
                } else if (segment.startsWith("{") && segment.endsWith("}")) {
                    params.put(segment.substring(1, segment.length() - 1), parts[i]);
                } else if (!segment.equals(parts[i])) {
                    return null;
                }
            }
            return params;
        }
    }

    private static final class Mount {
        final String prefix;
        final Function<Request, Router> router;

        Mount(String prefix, Function<Request, Router> router) {
            this.prefix = prefix;
            this.router = router;
        }

        boolean covers(String path) {
            return prefix.isEmpty() || path.equals(prefix) || path.startsWith(prefix + "/");
        }

        String strip(String path) {
            return path.substring(prefix.length());
        }
    }

    /** Either a route or a mount, in the order added. */
    private final List<Object> entries = new ArrayList<>();

    private Function<Request, Response> guard = request -> null;

    public Router route(String method, String pattern, Handler handler) {
        entries.add(new Route(method, pattern, handler));
        return this;
    }

    /** Serves {@code router}'s routes under {@code prefix}; an empty prefix mounts them here. */
    public Router mount(String prefix, Router router) {
        return mount(prefix, request -> router);
    }

    /** Serves, under {@code prefix}, the routes of whichever router {@code choose} picks for the request. */
    public Router mount(String prefix, Function<Request, Router> choose) {
        entries.add(new Mount(prefix, choose));
        return this;
    }

    /**
     * @param guard answers for every route and mount here, before any of them: a response refuses
     *              the request, null lets it through
     */
    public Router guard(Function<Request, Response> guard) {
        this.guard = guard;
        return this;
    }

    @Override
    public Response apply(Request request) {
        return handle(request);
    }

    public Response handle(Request request) {
        Response response = handle(request, request.path);
        return response != null ? response : Response.error(404, "not found: " + request.path);
    }

    /**
     * Whether a route here matches the path by any method, or a mount's prefix covers it. A
     * mount is known by its prefix alone, so the guard answers before the mounted router is even
     * chosen: choosing it may need what the guard checks.
     */
    private boolean knows(String path) {
        for (Object entry : entries) {
            if (entry instanceof Route ? ((Route) entry).match(path) != null : ((Mount) entry).covers(path)) {
                return true;
            }
        }
        return false;
    }

    /** The response, or null when nothing here knows the path. */
    private Response handle(Request request, String path) {
        if (!knows(path)) {
            return null;
        }
        Response refused = guard.apply(request);
        if (refused != null) {
            return refused;
        }
        List<String> allowed = new ArrayList<>();
        for (Object entry : entries) {
            if (entry instanceof Mount) {
                Mount mount = (Mount) entry;
                if (mount.covers(path)) {
                    Response response = mount.router.apply(request).handle(request, mount.strip(path));
                    if (response != null) {
                        return response;
                    }
                }
                continue;
            }
            Route route = (Route) entry;
            Map<String, String> params = route.match(path);
            if (params == null) {
                continue;
            }
            if (route.method.equals(request.method)) {
                return route.handler.handle(request, params);
            }
            allowed.add(route.method);
        }
        return allowed.isEmpty() ? null : Response.error(405, String.join(" or ", allowed) + " " + request.path);
    }
}
