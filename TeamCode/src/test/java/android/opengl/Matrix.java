package android.opengl;

public class Matrix {
    private static final int SIZE = 16;

    public static void setIdentityM(float[] sm, int smOffset) {
        for (int i = 0; i < SIZE; i++) {
            sm[smOffset + i] = i % 5 == 0 ? 1 : 0;
        }
    }

    public static void multiplyMM(
            float[] result, int resultOffset, float[] lhs, int lhsOffset, float[] rhs, int rhsOffset) {
        float[] product = new float[SIZE];
        for (int column = 0; column < 4; column++) {
            for (int row = 0; row < 4; row++) {
                double sum = 0;
                for (int k = 0; k < 4; k++) {
                    sum += (double) lhs[lhsOffset + row + 4 * k] * rhs[rhsOffset + k + 4 * column];
                }
                product[row + 4 * column] = (float) sum;
            }
        }
        System.arraycopy(product, 0, result, resultOffset, SIZE);
    }

    public static void setRotateM(float[] rm, int rmOffset, float a, float x, float y, float z) {
        double length = Math.sqrt((double) x * x + (double) y * y + (double) z * z);
        double ux = x / length;
        double uy = y / length;
        double uz = z / length;
        double radians = Math.toRadians(a);
        double s = Math.sin(radians);
        double c = Math.cos(radians);
        double nc = 1 - c;
        float[] r = {
            (float) (ux * ux * nc + c),
            (float) (ux * uy * nc + uz * s),
            (float) (uz * ux * nc - uy * s),
            0,
            (float) (ux * uy * nc - uz * s),
            (float) (uy * uy * nc + c),
            (float) (uy * uz * nc + ux * s),
            0,
            (float) (uz * ux * nc + uy * s),
            (float) (uy * uz * nc - ux * s),
            (float) (uz * uz * nc + c),
            0,
            0,
            0,
            0,
            1
        };
        System.arraycopy(r, 0, rm, rmOffset, SIZE);
    }

    public static void rotateM(float[] m, int mOffset, float a, float x, float y, float z) {
        rotateM(m, mOffset, m, mOffset, a, x, y, z);
    }

    public static void rotateM(float[] rm, int rmOffset, float[] m, int mOffset, float a, float x, float y, float z) {
        float[] rotation = new float[SIZE];
        setRotateM(rotation, 0, a, x, y, z);
        multiplyMM(rm, rmOffset, m, mOffset, rotation, 0);
    }

    public static void translateM(float[] m, int mOffset, float x, float y, float z) {
        translateM(m, mOffset, m, mOffset, x, y, z);
    }

    public static void translateM(float[] tm, int tmOffset, float[] m, int mOffset, float x, float y, float z) {
        float[] moved = new float[SIZE];
        System.arraycopy(m, mOffset, moved, 0, SIZE);
        for (int row = 0; row < 4; row++) {
            moved[12 + row] =
                    m[mOffset + row] * x + m[mOffset + 4 + row] * y + m[mOffset + 8 + row] * z + m[mOffset + 12 + row];
        }
        System.arraycopy(moved, 0, tm, tmOffset, SIZE);
    }

    public static void scaleM(float[] m, int mOffset, float x, float y, float z) {
        scaleM(m, mOffset, m, mOffset, x, y, z);
    }

    public static void scaleM(float[] sm, int smOffset, float[] m, int mOffset, float x, float y, float z) {
        float[] scaled = new float[SIZE];
        for (int row = 0; row < 4; row++) {
            scaled[row] = m[mOffset + row] * x;
            scaled[4 + row] = m[mOffset + 4 + row] * y;
            scaled[8 + row] = m[mOffset + 8 + row] * z;
            scaled[12 + row] = m[mOffset + 12 + row];
        }
        System.arraycopy(scaled, 0, sm, smOffset, SIZE);
    }

    public static boolean invertM(float[] mInv, int mInvOffset, float[] m, int mOffset) {
        double[] a = new double[SIZE];
        for (int i = 0; i < SIZE; i++) {
            a[i] = m[mOffset + i];
        }
        double[] inv = new double[SIZE];
        inv[0] = a[5] * a[10] * a[15]
                - a[5] * a[11] * a[14]
                - a[9] * a[6] * a[15]
                + a[9] * a[7] * a[14]
                + a[13] * a[6] * a[11]
                - a[13] * a[7] * a[10];
        inv[4] = -a[4] * a[10] * a[15]
                + a[4] * a[11] * a[14]
                + a[8] * a[6] * a[15]
                - a[8] * a[7] * a[14]
                - a[12] * a[6] * a[11]
                + a[12] * a[7] * a[10];
        inv[8] = a[4] * a[9] * a[15]
                - a[4] * a[11] * a[13]
                - a[8] * a[5] * a[15]
                + a[8] * a[7] * a[13]
                + a[12] * a[5] * a[11]
                - a[12] * a[7] * a[9];
        inv[12] = -a[4] * a[9] * a[14]
                + a[4] * a[10] * a[13]
                + a[8] * a[5] * a[14]
                - a[8] * a[6] * a[13]
                - a[12] * a[5] * a[10]
                + a[12] * a[6] * a[9];
        inv[1] = -a[1] * a[10] * a[15]
                + a[1] * a[11] * a[14]
                + a[9] * a[2] * a[15]
                - a[9] * a[3] * a[14]
                - a[13] * a[2] * a[11]
                + a[13] * a[3] * a[10];
        inv[5] = a[0] * a[10] * a[15]
                - a[0] * a[11] * a[14]
                - a[8] * a[2] * a[15]
                + a[8] * a[3] * a[14]
                + a[12] * a[2] * a[11]
                - a[12] * a[3] * a[10];
        inv[9] = -a[0] * a[9] * a[15]
                + a[0] * a[11] * a[13]
                + a[8] * a[1] * a[15]
                - a[8] * a[3] * a[13]
                - a[12] * a[1] * a[11]
                + a[12] * a[3] * a[9];
        inv[13] = a[0] * a[9] * a[14]
                - a[0] * a[10] * a[13]
                - a[8] * a[1] * a[14]
                + a[8] * a[2] * a[13]
                + a[12] * a[1] * a[10]
                - a[12] * a[2] * a[9];
        inv[2] = a[1] * a[6] * a[15]
                - a[1] * a[7] * a[14]
                - a[5] * a[2] * a[15]
                + a[5] * a[3] * a[14]
                + a[13] * a[2] * a[7]
                - a[13] * a[3] * a[6];
        inv[6] = -a[0] * a[6] * a[15]
                + a[0] * a[7] * a[14]
                + a[4] * a[2] * a[15]
                - a[4] * a[3] * a[14]
                - a[12] * a[2] * a[7]
                + a[12] * a[3] * a[6];
        inv[10] = a[0] * a[5] * a[15]
                - a[0] * a[7] * a[13]
                - a[4] * a[1] * a[15]
                + a[4] * a[3] * a[13]
                + a[12] * a[1] * a[7]
                - a[12] * a[3] * a[5];
        inv[14] = -a[0] * a[5] * a[14]
                + a[0] * a[6] * a[13]
                + a[4] * a[1] * a[14]
                - a[4] * a[2] * a[13]
                - a[12] * a[1] * a[6]
                + a[12] * a[2] * a[5];
        inv[3] = -a[1] * a[6] * a[11]
                + a[1] * a[7] * a[10]
                + a[5] * a[2] * a[11]
                - a[5] * a[3] * a[10]
                - a[9] * a[2] * a[7]
                + a[9] * a[3] * a[6];
        inv[7] = a[0] * a[6] * a[11]
                - a[0] * a[7] * a[10]
                - a[4] * a[2] * a[11]
                + a[4] * a[3] * a[10]
                + a[8] * a[2] * a[7]
                - a[8] * a[3] * a[6];
        inv[11] = -a[0] * a[5] * a[11]
                + a[0] * a[7] * a[9]
                + a[4] * a[1] * a[11]
                - a[4] * a[3] * a[9]
                - a[8] * a[1] * a[7]
                + a[8] * a[3] * a[5];
        inv[15] = a[0] * a[5] * a[10]
                - a[0] * a[6] * a[9]
                - a[4] * a[1] * a[10]
                + a[4] * a[2] * a[9]
                + a[8] * a[1] * a[6]
                - a[8] * a[2] * a[5];
        double determinant = a[0] * inv[0] + a[1] * inv[4] + a[2] * inv[8] + a[3] * inv[12];
        if (determinant == 0) {
            return false;
        }
        for (int i = 0; i < SIZE; i++) {
            mInv[mInvOffset + i] = (float) (inv[i] / determinant);
        }
        return true;
    }
}
