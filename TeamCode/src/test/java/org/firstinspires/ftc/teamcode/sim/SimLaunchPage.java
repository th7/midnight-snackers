package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;

final class SimLaunchPage {
    static final String TEMPLATE = "launch.html";
    static final String TITLE = "Launch Sim";
    static final String ASSETS_FROM_THE_VIEWER = "../assets/";

    private static final Gson GSON = new GsonBuilder().serializeNulls().create();

    private SimLaunchPage() {}

    static String viewer() {
        return template()
                .replace("__IMPORTMAP__", importMap(ASSETS_FROM_THE_VIEWER))
                .replace("__ASSETS__", GSON.toJson(ASSETS_FROM_THE_VIEWER))
                .replace("__TITLE__", TITLE)
                .replace("__FIELD__", GSON.toJson(SimPlacement.FIELD_JSON))
                .replace("__DATA__", "{}");
    }

    private static String importMap(String assetsUnder) {
        return "<script type=\"importmap\">\n{\"imports\": {\"three\": \"" + assetsUnder
                + "vendor/three.module.min.js\", \"three/addons/\": \"" + assetsUnder + "vendor/jsm/\"}}\n</script>";
    }

    private static String template() {
        try (InputStream in = SimLaunchPage.class.getResourceAsStream(TEMPLATE)) {
            if (in == null) {
                throw new IllegalStateException(
                        "missing resource " + TEMPLATE + " next to " + SimLaunchPage.class.getName());
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
