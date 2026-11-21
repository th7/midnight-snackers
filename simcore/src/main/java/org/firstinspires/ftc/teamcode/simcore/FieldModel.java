package org.firstinspires.ftc.teamcode.simcore;

public enum FieldModel {
    LOW("low", "field.glb"),
    MEDIUM("medium", "field-medium.glb"),
    HIGH("high", "field-high.glb");

    private final String asked;
    private final String file;

    FieldModel(String asked, String file) {
        this.asked = asked;
        this.file = file;
    }

    public String asked() {
        return asked;
    }

    public String file() {
        return file;
    }

    public static Checked<FieldModel> named(String asked) {
        for (FieldModel one : values()) {
            if (one.asked.equals(asked)) {
                return Checked.ok(one);
            }
        }
        return Checked.rejected("no field model is named " + asked + ": they are low, medium or high");
    }
}
