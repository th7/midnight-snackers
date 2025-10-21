package org.firstinspires.ftc.teamcode;

public class MoveData {
    public float frontLeftPower = 0f;
    public float frontRightPower = 0f;
    public float rearLeftPower = 0f;
    public float rearRightPower = 0f;

    public static MoveData straight(float power, float min, float max) {
        MoveData movement = new MoveData();
        movement.straight(power);
        movement.min(min);
        movement.max(max);
        return movement;
    }

    public static MoveData turn(float power, float min, float max) {
        MoveData movement = new MoveData();
        movement.turn(power);
        movement.min(min);
        movement.max(max);
        return movement;
    }

    public static MoveData strafe(float power, float min, float max) {
        MoveData movement = new MoveData();
        movement.strafe(power);
        movement.min(min);
        movement.max(max);
        return movement;
    }

    public MoveData add(MoveData... movements) {
        MoveData movement = new MoveData();

        movement.frontLeftPower = this.frontLeftPower;
        movement.frontRightPower = this.frontRightPower;
        movement.rearLeftPower = this.rearLeftPower;
        movement.rearRightPower = this.rearRightPower;

        for (MoveData other : movements) {
            movement.frontLeftPower += other.frontLeftPower;
            movement.frontRightPower += other.frontRightPower;
            movement.rearLeftPower += other.rearLeftPower;
            movement.rearRightPower += other.rearRightPower;
        }

        movement.min(0f);
        movement.max(1f);
        return movement;
    }

    private void straight(float power) {
        this.frontLeftPower = power;
        this.frontRightPower = power;
        this.rearLeftPower = power;
        this.rearRightPower = power;
    }

    private void turn(float power) {
        this.frontLeftPower = -power;
        this.frontRightPower = power;
        this.rearLeftPower = -power;
        this.rearRightPower = power;
    }

    private void strafe(float power) {
        this.frontLeftPower = -power;
        this.frontRightPower = power;
        this.rearLeftPower = power;
        this.rearRightPower = -power;
    }

    private void max(float max) {
        this.frontLeftPower = max(this.frontLeftPower, max);
        this.frontRightPower = max(this.frontRightPower, max);
        this.rearRightPower = max(this.rearRightPower, max);
        this.rearLeftPower = max(this.rearLeftPower, max);
    }

    private void min(float min) {
        this.frontLeftPower = min(this.frontLeftPower, min);
        this.frontRightPower = min(this.frontRightPower, min);
        this.rearRightPower = min(this.rearRightPower, min);
        this.rearLeftPower = min(this.rearLeftPower, min);
    }

    private float max(float unclipped, float max) {
        if (unclipped < -max) {
            return -max;
        }

        return Math.min(unclipped, max);
    }

    private float min(float unclipped, float min) {
        if (-min < unclipped && unclipped < 0) {
            return -min;
        }

        if (0 < unclipped && unclipped < min) {
            return min;
        }

        return unclipped;
    }
}
