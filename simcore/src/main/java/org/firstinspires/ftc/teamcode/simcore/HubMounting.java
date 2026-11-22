package org.firstinspires.ftc.teamcode.simcore;

public final class HubMounting {
    private final Facing logo;
    private final Facing usb;

    private HubMounting(Facing logo, Facing usb) {
        this.logo = logo;
        this.usb = usb;
    }

    public static HubMounting reginalds() {
        return new HubMounting(Facing.RIGHT, Facing.UP);
    }

    public static Checked<HubMounting> of(Facing logo, Facing usb) {
        if (logo == usb || logo.opposite() == usb) {
            return Checked.rejected("the hub's logo and its USB ports face along different axes, not UP and DOWN,"
                    + " not LEFT and RIGHT, not FORWARD and BACKWARD: the logo faces " + logo + " and the USB ports "
                    + usb);
        }
        return Checked.ok(new HubMounting(logo, usb));
    }

    public Facing logo() {
        return logo;
    }

    public Facing usb() {
        return usb;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof HubMounting that && logo == that.logo && usb == that.usb;
    }

    @Override
    public int hashCode() {
        return 31 * logo.hashCode() + usb.hashCode();
    }

    @Override
    public String toString() {
        return "logo " + logo + ", USB " + usb;
    }
}
