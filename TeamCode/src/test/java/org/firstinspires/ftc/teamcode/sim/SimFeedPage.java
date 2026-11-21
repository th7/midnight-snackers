package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;

final class SimFeedPage {
    static final String TEMPLATE = "feeder.html";
    static final String TITLE = "Feeder Sim";
    static final String ASSETS_FROM_THE_VIEWER = "../assets/";

    private SimFeedPage() {}

    static String viewer() {
        return template()
                .replace("__IMPORTMAP__", SimLaunchPage.importMap(ASSETS_FROM_THE_VIEWER))
                .replace("__ASSETS__", new Gson().toJson(ASSETS_FROM_THE_VIEWER))
                .replace("__TITLE__", TITLE);
    }

    private static String template() {
        try (InputStream in = SimFeedPage.class.getResourceAsStream(TEMPLATE)) {
            if (in == null) {
                throw new IllegalStateException(
                        "missing resource " + TEMPLATE + " next to " + SimFeedPage.class.getName());
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
