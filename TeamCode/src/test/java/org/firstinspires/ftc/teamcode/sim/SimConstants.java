package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonArray;
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

    public static JsonObject listing(Constants constants) {
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
            listed.add(one);
        }
        JsonObject listing = new JsonObject();
        listing.add("groups", groups);
        listing.add("constants", listed);
        return listing;
    }

    public static JsonObject changed(Constants constants) {
        JsonObject changed = new JsonObject();
        for (Map.Entry<Constant, Double> entry : constants.changed().entrySet()) {
            changed.addProperty(entry.getKey().asked(), entry.getValue());
        }
        return changed;
    }

    public static Checked<Constants> fromJson(JsonElement element) {
        if (element == null || element.isJsonNull()) {
            return Checked.ok(Constants.defaults());
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
        return Constants.of(set);
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
