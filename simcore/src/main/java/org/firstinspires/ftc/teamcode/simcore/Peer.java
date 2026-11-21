package org.firstinspires.ftc.teamcode.simcore;

public final class Peer {
    private final String address;

    private Peer(String address) {
        this.address = address;
    }

    public static Checked<Peer> of(String address) {
        return address.isBlank()
                ? Checked.rejected("a peer is the address a connection came from, and no address was recorded")
                : Checked.ok(new Peer(address));
    }

    public String address() {
        return address;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Peer that && address.equals(that.address);
    }

    @Override
    public int hashCode() {
        return address.hashCode();
    }

    @Override
    public String toString() {
        return address;
    }
}
