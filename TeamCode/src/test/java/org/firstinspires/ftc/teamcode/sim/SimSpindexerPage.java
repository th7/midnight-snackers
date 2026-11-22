package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;

final class SimSpindexerPage {
    static final String TEMPLATE = "spindexer.html";
    static final String TITLE = "Spindexer Sim";
    static final String ASSETS_FROM_THE_VIEWER = "../assets/";

    private SimSpindexerPage() {}

    static String viewer() {
        return template()
                .replace("__IMPORTMAP__", SimLaunchPage.importMap(ASSETS_FROM_THE_VIEWER))
                .replace("__ASSETS__", new Gson().toJson(ASSETS_FROM_THE_VIEWER))
                .replace("__TITLE__", TITLE);
    }

    private static String template() {
        try (InputStream in = SimSpindexerPage.class.getResourceAsStream(TEMPLATE)) {
            if (in == null) {
                throw new IllegalStateException(
                        "missing resource " + TEMPLATE + " next to " + SimSpindexerPage.class.getName());
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
