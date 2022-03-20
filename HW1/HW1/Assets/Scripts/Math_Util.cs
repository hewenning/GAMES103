using UnityEngine;

namespace Rigid
{
    public static class MathUtil
    {
        public static Vector3 gravity = new Vector3(0, -9.8f, 0);


        public static Matrix4x4 Add(this Matrix4x4 A, Matrix4x4 B)
        {
            Matrix4x4 M = Matrix4x4.zero;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    M[i, j] = A[i, j] + B[i, j];
                }
            }
            return M;
        }

        public static Matrix4x4 Subtract(this Matrix4x4 A, Matrix4x4 B)
        {
            Matrix4x4 M = Matrix4x4.zero;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    M[i, j] = A[i, j] - B[i, j];
                }
            }
            return M;
        }

        public static Matrix4x4 Scale(this Matrix4x4 lhs, float s)
        {
            Matrix4x4 retMatrix;
            retMatrix.m00 = lhs.m00 * s;
            retMatrix.m01 = lhs.m01 * s;
            retMatrix.m02 = lhs.m02 * s;
            retMatrix.m03 = lhs.m03 * s;
            retMatrix.m10 = lhs.m10 * s;
            retMatrix.m11 = lhs.m11 * s;
            retMatrix.m12 = lhs.m12 * s;
            retMatrix.m13 = lhs.m13 * s;
            retMatrix.m20 = lhs.m20 * s;
            retMatrix.m21 = lhs.m21 * s;
            retMatrix.m22 = lhs.m22 * s;
            retMatrix.m23 = lhs.m23 * s;
            retMatrix.m30 = lhs.m30 * s;
            retMatrix.m31 = lhs.m31 * s;
            retMatrix.m32 = lhs.m32 * s;
            retMatrix.m33 = lhs.m33 * s;
            return retMatrix;
        }

        public static Matrix4x4 ColMultiply(this Vector3 a, Vector3 b)
        {
            Matrix4x4 retMatrix = Matrix4x4.zero;
            retMatrix.m00 = a.x * b.x;
            retMatrix.m01 = a.x * b.y;
            retMatrix.m02 = a.x * b.z;
            retMatrix.m10 = a.y * b.x;
            retMatrix.m11 = a.y * b.y;
            retMatrix.m12 = a.y * b.z;
            retMatrix.m20 = a.z * b.x;
            retMatrix.m21 = a.z * b.y;
            retMatrix.m22 = a.z * b.z;
            retMatrix.m33 = 1;
            return retMatrix;
        }

        public static Matrix4x4 GetCrossMatrix(Vector3 a)
        {
            //Get the cross product matrix of vector a
            Matrix4x4 A = Matrix4x4.zero;
            A[0, 0] = 0;
            A[0, 1] = -a[2];
            A[0, 2] = a[1];
            A[1, 0] = a[2];
            A[1, 1] = 0;
            A[1, 2] = -a[0];
            A[2, 0] = -a[1];
            A[2, 1] = a[0];
            A[2, 2] = 0;
            A[3, 3] = 1;
            return A;
        }

        public static Quaternion QuatAdd(Quaternion a, Quaternion b)
        {
            Quaternion q = new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
            return q;
        }

        // Polar Decomposition that returns the rotation from F.
        public static Matrix4x4 GetRotation(Matrix4x4 F)
        {
            Matrix4x4 C = Matrix4x4.zero;
            for (int ii = 0; ii < 3; ii++)
                for (int jj = 0; jj < 3; jj++)
                    for (int kk = 0; kk < 3; kk++)
                        C[ii, jj] += F[kk, ii] * F[kk, jj];

            Matrix4x4 C2 = Matrix4x4.zero;
            for (int ii = 0; ii < 3; ii++)
                for (int jj = 0; jj < 3; jj++)
                    for (int kk = 0; kk < 3; kk++)
                        C2[ii, jj] += C[ii, kk] * C[jj, kk];

            float det = F[0, 0] * F[1, 1] * F[2, 2] +
                            F[0, 1] * F[1, 2] * F[2, 0] +
                            F[1, 0] * F[2, 1] * F[0, 2] -
                            F[0, 2] * F[1, 1] * F[2, 0] -
                            F[0, 1] * F[1, 0] * F[2, 2] -
                            F[0, 0] * F[1, 2] * F[2, 1];

            float I_c = C[0, 0] + C[1, 1] + C[2, 2];
            float I_c2 = I_c * I_c;
            float II_c = 0.5f * (I_c2 - C2[0, 0] - C2[1, 1] - C2[2, 2]);
            float III_c = det * det;
            float k = I_c2 - 3 * II_c;

            Matrix4x4 inv_U = Matrix4x4.zero;
            if (k < 1e-10f)
            {
                float inv_lambda = 1 / Mathf.Sqrt(I_c / 3);
                inv_U[0, 0] = inv_lambda;
                inv_U[1, 1] = inv_lambda;
                inv_U[2, 2] = inv_lambda;
            }
            else
            {
                float l = I_c * (I_c * I_c - 4.5f * II_c) + 13.5f * III_c;
                float k_root = Mathf.Sqrt(k);
                float value = l / (k * k_root);
                if (value < -1.0f) value = -1.0f;
                if (value > 1.0f) value = 1.0f;
                float phi = Mathf.Acos(value);
                float lambda2 = (I_c + 2 * k_root * Mathf.Cos(phi / 3)) / 3.0f;
                float lambda = Mathf.Sqrt(lambda2);

                float III_u = Mathf.Sqrt(III_c);
                if (det < 0) III_u = -III_u;
                float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2 * III_u / lambda);
                float II_u = (I_u * I_u - I_c) * 0.5f;


                float inv_rate, factor;
                inv_rate = 1 / (I_u * II_u - III_u);
                factor = I_u * III_u * inv_rate;

                Matrix4x4 U = Matrix4x4.zero;
                U[0, 0] = factor;
                U[1, 1] = factor;
                U[2, 2] = factor;

                factor = (I_u * I_u - II_u) * inv_rate;
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        U[i, j] += factor * C[i, j] - inv_rate * C2[i, j];

                inv_rate = 1 / III_u;
                factor = II_u * inv_rate;
                inv_U[0, 0] = factor;
                inv_U[1, 1] = factor;
                inv_U[2, 2] = factor;

                factor = -I_u * inv_rate;
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        inv_U[i, j] += factor * U[i, j] + inv_rate * C[i, j];
            }

            Matrix4x4 R = Matrix4x4.zero;
            for (int ii = 0; ii < 3; ii++)
                for (int jj = 0; jj < 3; jj++)
                    for (int kk = 0; kk < 3; kk++)
                        R[ii, jj] += F[ii, kk] * inv_U[kk, jj];
            R[3, 3] = 1;
            return R;
        }

    }
}