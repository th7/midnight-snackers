package android.opengl;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.junit.Test;

public class MatrixTest {
    private static final String SDK_CHIP_TURN =
            "the SDK's RevImuOrientationOnRobot (Hardware 11.2.1) turns every mounting -90 degrees about z"
                    + " for the chip inside the hub, and USB LEFT is +90 about z";
    private static final String ANDROID_MATRIX = "https://developer.android.com/reference/android/opengl/Matrix";
    private static final float DELTA = 1e-6f;

    private static float[] identity() {
        float[] m = new float[16];
        Matrix.setIdentityM(m, 0);
        return m;
    }

    private static float[] times(float[] m, float x, float y, float z) {
        float[] v = new float[4];
        for (int row = 0; row < 4; row++) {
            v[row] = m[row] * x + m[4 + row] * y + m[8 + row] * z + m[12 + row];
        }
        return v;
    }

    @Test
    public void theIdentityHasOnesDownItsDiagonalAndNothingElse() {
        assertArrayEquals(ANDROID_MATRIX, new float[] {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, identity(), 0);
    }

    @Test
    public void aMatrixIsSetAtItsOffsetAndNowhereElse() {
        float[] m = new float[20];
        java.util.Arrays.fill(m, 9);

        Matrix.setIdentityM(m, 2);

        assertEquals(9, m[0], 0);
        assertEquals(9, m[1], 0);
        assertEquals(1, m[2], 0);
        assertEquals(0, m[3], 0);
        assertEquals(1, m[17], 0);
        assertEquals(9, m[18], 0);
    }

    @Test
    public void aRotationIsInDegreesAntiClockwiseAboutItsAxisColumnByColumn() {
        float[] m = new float[16];

        Matrix.setRotateM(m, 0, 90, 0, 0, 1);

        assertArrayEquals(ANDROID_MATRIX, new float[] {0, 1, 0, 1}, times(m, 1, 0, 0), DELTA);
        assertArrayEquals(ANDROID_MATRIX, new float[] {-1, 0, 0, 1}, times(m, 0, 1, 0), DELTA);
        Matrix.setRotateM(m, 0, 90, 0, 0, 2);
        assertArrayEquals("its axis need not be a unit", new float[] {0, 1, 0, 1}, times(m, 1, 0, 0), DELTA);
    }

    @Test
    public void aProductAppliesItsRightHandMatrixFirst() {
        float[] turn = new float[16];
        Matrix.setRotateM(turn, 0, 90, 0, 0, 1);
        float[] move = identity();
        Matrix.translateM(move, 0, 5, 0, 0);
        float[] product = new float[16];

        Matrix.multiplyMM(product, 0, move, 0, turn, 0);

        assertArrayEquals(ANDROID_MATRIX, new float[] {5, 1, 0, 1}, times(product, 1, 0, 0), DELTA);
    }

    @Test
    public void rotatingAMatrixTurnsWhatItIsAppliedToBeforeTheMatrixActs() {
        float[] m = identity();
        Matrix.translateM(m, 0, 5, 0, 0);
        float[] into = new float[16];

        Matrix.rotateM(into, 0, m, 0, 90, 0, 0, 1);
        Matrix.rotateM(m, 0, 90, 0, 0, 1);

        assertArrayEquals(ANDROID_MATRIX, new float[] {5, 1, 0, 1}, times(m, 1, 0, 0), DELTA);
        assertArrayEquals(ANDROID_MATRIX, m, into, DELTA);
    }

    @Test
    public void translatingAMatrixMovesWhatItIsAppliedToBeforeTheMatrixActs() {
        float[] m = new float[16];
        Matrix.setRotateM(m, 0, 90, 0, 0, 1);
        float[] into = new float[16];

        Matrix.translateM(into, 0, m, 0, 1, 0, 0);
        Matrix.translateM(m, 0, 1, 0, 0);

        assertArrayEquals(ANDROID_MATRIX, new float[] {0, 2, 0, 1}, times(m, 1, 0, 0), DELTA);
        assertArrayEquals(ANDROID_MATRIX, m, into, DELTA);
    }

    @Test
    public void scalingAMatrixStretchesWhatItIsAppliedToBeforeTheMatrixActs() {
        float[] m = identity();
        Matrix.translateM(m, 0, 1, 0, 0);
        float[] into = new float[16];

        Matrix.scaleM(into, 0, m, 0, 2, 3, 4);
        Matrix.scaleM(m, 0, 2, 3, 4);

        assertArrayEquals(ANDROID_MATRIX, new float[] {3, 3, 4, 1}, times(m, 1, 1, 1), DELTA);
        assertArrayEquals(ANDROID_MATRIX, m, into, DELTA);
    }

    @Test
    public void anInverseUndoesItsMatrixAndASingularOneHasNone() {
        float[] m = new float[16];
        Matrix.setRotateM(m, 0, 30, 1, 2, 3);
        Matrix.translateM(m, 0, 4, 5, 6);
        Matrix.scaleM(m, 0, 2, 2, 2);
        float[] inverse = new float[16];
        float[] product = new float[16];

        assertTrue(Matrix.invertM(inverse, 0, m, 0));
        Matrix.multiplyMM(product, 0, inverse, 0, m, 0);

        assertArrayEquals(ANDROID_MATRIX, identity(), product, 1e-5f);
        assertFalse(Matrix.invertM(inverse, 0, new float[16], 0));
    }

    private static Quaternion mounted(RevHubOrientationOnRobot.UsbFacingDirection usb) {
        return new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, usb)
                .imuCoordinateSystemOrientationFromPerspectiveOfRobot();
    }

    @Test
    public void theSdkWorksOutAHubsMountingWithItAroundTheChipsQuarterTurnInsideTheHub() {
        Quaternion usbLeft = mounted(RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        Quaternion usbForward = mounted(RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        assertEquals(SDK_CHIP_TURN, 1, Math.abs(usbLeft.w), DELTA);
        assertEquals(SDK_CHIP_TURN, 0, usbLeft.z, DELTA);
        assertEquals(SDK_CHIP_TURN, 0.70710678, Math.abs(usbForward.w), DELTA);
        assertEquals(SDK_CHIP_TURN, 0, usbForward.x, DELTA);
        assertEquals(SDK_CHIP_TURN, 0, usbForward.y, DELTA);
        assertEquals(SDK_CHIP_TURN, 0.70710678, Math.abs(usbForward.z), DELTA);
    }
}
