package org.firstinspires.ftc.teamcode.simcore;

import java.util.Locale;

public enum Surface {
    FLOOR("Floor", "The soft foam tiles the field is laid with."),
    WALL("Walls", "The perimeter: polycarbonate panels between aluminium rails."),
    FRAME("Frame", "Metal: the A-frame the hives hang from, and the hives' own tubes, brackets and tag plates."),
    FLOWER("Flowers", "The flowers' HIPS pipes and the plates that hold them."),
    CELL("Cells", "The walls and backs of the hives' cells, which a scored ball lands in."),
    PIECE(
            "Pieces",
            "Another ball: polyethylene on polyethylene, when the launched pollen meets a piece or two pieces meet.");

    private final String label;
    private final String says;

    Surface(String label, String says) {
        this.label = label;
        this.says = says;
    }

    public String label() {
        return label;
    }

    public String says() {
        return says;
    }

    public String asked() {
        return name().toLowerCase(Locale.ROOT);
    }
}
