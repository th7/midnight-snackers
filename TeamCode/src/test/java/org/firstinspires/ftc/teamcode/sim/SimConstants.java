package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.Constants;

public final class SimConstants {
    private SimConstants() {}

    public static JsonObject listing(Constants constants) {
        return listing(Constants.Restored.asSet(constants));
    }

    public static JsonObject listing(Constants.Restored restored) {
        Constants constants = restored.constants();
        Map<Constant, Double> pastTheirRange = restored.pastTheirRange();
        JsonArray groups = new JsonArray();
        for (Constant.Group group : Constant.Group.values()) {
            JsonObject one = new JsonObject();
            one.addProperty("name", group.asked());
            one.addProperty("label", group.label());
            one.addProperty("says", group.says());
            groups.add(one);
        }
        JsonArray listed = new JsonArray();
        for (Constant constant : Constant.values()) {
            JsonObject one = new JsonObject();
            one.addProperty("name", constant.asked());
            one.addProperty("group", constant.group().asked());
            one.addProperty("label", constant.label());
            one.addProperty("unit", constant.unit());
            one.addProperty("says", constant.says());
            one.addProperty("least", constant.least());
            one.addProperty("most", constant.most());
            one.addProperty("byDefault", constant.byDefault());
            one.addProperty("value", constants.value(constant));
            if (pastTheirRange.containsKey(constant)) {
                one.addProperty("saved", pastTheirRange.get(constant));
            }
            listed.add(one);
        }
        JsonObject listing = new JsonObject();
        listing.add("groups", groups);
        listing.add("constants", listed);
        Map<Constant.Retired, Double> setAside = restored.setAside();
        if (!setAside.isEmpty()) {
            JsonArray dropped = new JsonArray();
            for (Map.Entry<Constant.Retired, Double> entry : setAside.entrySet()) {
                JsonObject one = new JsonObject();
                one.addProperty("name", entry.getKey().asked());
                one.addProperty("label", entry.getKey().label());
                one.addProperty("saved", entry.getValue());
                one.addProperty("why", entry.getKey().why());
                dropped.add(one);
            }
            listing.add("dropped", dropped);
        }
        return listing;
    }

    public static JsonObject changed(Constants constants) {
        return numbersByName(constants.changed());
    }

    public static JsonObject saved(Constants.Restored restored) {
        JsonObject saved = numbersByName(restored.asSaved());
        for (Map.Entry<Constant.Retired, Double> entry : restored.setAside().entrySet()) {
            saved.addProperty(entry.getKey().asked(), entry.getValue());
        }
        return saved;
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

    public static Checked<Constants.Restored> restoredFromJson(JsonElement element) {
        if (element == null || !element.isJsonObject()) {
            return numbersByName(element).then(Constants::restored);
        }
        JsonObject saved = new JsonObject();
        Map<Constant.Retired, Double> retired = new EnumMap<>(Constant.Retired.class);
        for (Map.Entry<String, JsonElement> entry : element.getAsJsonObject().entrySet()) {
            Optional<Constant.Retired> gone = Constant.Retired.named(entry.getKey());
            if (gone.isEmpty()) {
                saved.add(entry.getKey(), entry.getValue());
                continue;
            }
            JsonElement value = entry.getValue();
            if (!(value.isJsonPrimitive() && value.getAsJsonPrimitive().isNumber())) {
                return Checked.rejected(gone.get().label() + " is a number, not " + value);
            }
            retired.put(gone.get(), value.getAsDouble());
        }
        return numbersByName(saved).then(constants -> Constants.restored(constants, retired));
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
        return described(constants.changed());
    }

    public static String described(Map<Constant, Double> changed) {
        List<String> changes = new ArrayList<>();
        for (Map.Entry<Constant, Double> entry : changed.entrySet()) {
            Constant constant = entry.getKey();
            changes.add(constant.label() + " " + entry.getValue() + " " + constant.unit() + " (built "
                    + constant.byDefault() + ")");
        }
        return String.join("; ", changes);
    }
}
