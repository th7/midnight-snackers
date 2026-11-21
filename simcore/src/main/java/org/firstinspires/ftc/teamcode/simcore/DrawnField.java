package org.firstinspires.ftc.teamcode.simcore;

import java.util.Set;

public sealed interface DrawnField permits DrawnField.Committed, DrawnField.Kept {
    static DrawnField of(Set<FieldModel> kept) {
        DrawnField drawn = new Committed();
        for (FieldModel one : FieldModel.values()) {
            if (kept.contains(one)) {
                drawn = new Kept(one);
            }
        }
        return drawn;
    }

    FieldModel model();

    String said();

    record Committed() implements DrawnField {
        @Override
        public FieldModel model() {
            return FieldModel.LOW;
        }

        @Override
        public String said() {
            return "The low field committed with the project: this computer has kept none of the host's.";
        }
    }

    record Kept(FieldModel model) implements DrawnField {
        @Override
        public String said() {
            return "The " + model.asked() + " field the host built, kept on this computer.";
        }
    }
}
