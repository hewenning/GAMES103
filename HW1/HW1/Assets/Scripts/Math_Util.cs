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
    }
}