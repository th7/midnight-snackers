package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

public final class Router implements Function<Request, Response> {
    public interface Handler {
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

    private final List<Object> entries = new ArrayList<>();

    private Function<Request, Response> guard = request -> null;

    public Router route(String method, String pattern, Handler handler) {
        entries.add(new Route(method, pattern, handler));
        return this;
    }

    public Router redirect(String method, String pattern, Function<Map<String, String>, String> target) {
        return route(
                method,
                pattern,
                (request, params) -> new Response(308, "text/plain; charset=utf-8", "")
                        .withHeader("Location", target.apply(params)));
    }

    public Router mount(String prefix, Router router) {
        return mount(prefix, request -> router);
    }

    public Router mount(String prefix, Function<Request, Router> choose) {
        entries.add(new Mount(prefix, choose));
        return this;
    }

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

    private boolean knows(String path) {
        for (Object entry : entries) {
            if (entry instanceof Route ? ((Route) entry).match(path) != null : ((Mount) entry).covers(path)) {
                return true;
            }
        }
        return false;
    }

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
