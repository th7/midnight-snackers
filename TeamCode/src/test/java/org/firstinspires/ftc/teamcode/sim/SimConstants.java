package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.Constants;

public final class SimConstants {
    private SimConstants() {}

    public static JsonObject changed(Constants constants) {
        return numbersByName(constants.changed());
    }

    private static JsonObject numbersByName(Map<Constant, Double> numbers) {
        JsonObject byName = new JsonObject();
        for (Map.Entry<Constant, Double> entry : numbers.entrySet()) {
            byName.addProperty(entry.getKey().asked(), entry.getValue());
        }
        return byName;
    }

    public static Checked<Constants> fromJson(JsonElement element) {
        return numbersByName(element).then(Constants::of);
    }

    private static Checked<Map<Constant, Double>> numbersByName(JsonElement element) {
        if (element == null || element.isJsonNull()) {
            return Checked.ok(Map.of());
        }
        if (!element.isJsonObject()) {
            return Checked.rejected("the simulation constants are an object of numbers by name, not " + element);
        }
        Map<Constant, Double> set = new EnumMap<>(Constant.class);
        for (Map.Entry<String, JsonElement> entry : element.getAsJsonObject().entrySet()) {
            Checked<Constant> named = Constant.named(entry.getKey());
            if (named instanceof Checked.Rejected<Constant> rejected) {
                return Checked.rejected(rejected.rule());
            }
            Constant constant = Valid.value(named);
            JsonElement value = entry.getValue();
            if (!(value.isJsonPrimitive() && value.getAsJsonPrimitive().isNumber())) {
                return Checked.rejected(constant.label() + " is a number, not " + value);
            }
            set.put(constant, value.getAsDouble());
        }
        return Checked.ok(set);
    }

    public static String described(Constants constants) {
        if (constants.asBuilt()) {
            return "as built";
        }
        List<String> changes = new ArrayList<>();
        for (Map.Entry<Constant, Double> entry : constants.changed().entrySet()) {
            Constant constant = entry.getKey();
            changes.add(constant.label() + " " + entry.getValue() + " " + constant.unit() + " (built "
                    + constant.byDefault() + ")");
        }
        return String.join("; ", changes);
    }
}
