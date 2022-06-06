using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Complex;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotModelTool : MonoBehaviour
{
    /// <summary>
    /// Get Homogenous Matrix M 
    /// </summary>
    public Matrix4x4 GetHomoMatrix(float[] theta, float[] d, float[] a, float[] alpha)
    {
        Matrix4x4 m = Matrix4x4.identity;
        for (int i = 0; i < theta.Length; i++)
            m *= GetTransMatrix(theta[i], d[i], a[i], alpha[i]);
        return m;
    }

    /// <summary>
    /// Get Skew-symmetric Matrix of a vector
    /// </summary>
    public Matrix<float> SkewSymmetric(Vector<float> v)
    {
        int n = v.Count;
        var mm = DenseMatrix.Create(n, n, 0);
        var m = Matrix<float>.Build.Dense(n, n);
        var vv = Vector<float>.Build.Dense(3);
        m[0, 1] = 1;
        (vv[0], vv[1], vv[2]) = (0, -v[2], v[1]);
        m.SetRow(0, vv);
        (vv[0], vv[1], vv[2]) = (v[2], 0, -v[0]);
        m.SetRow(1, vv);
        (vv[0], vv[1], vv[2]) = (-v[1], v[0], 0);
        m.SetRow(2, vv);
        return m;
    }

    /// <summary>
    /// Get T Matrix with Joint DH parameter
    /// </summary>
    public Matrix4x4 GetTransMatrix(float theta, float d, float a, float alpha)
    {
        Matrix4x4 T = new Matrix4x4();
        T.SetRow(0, new Vector4(Mathf.Cos(theta), -Mathf.Sin(theta) * Mathf.Cos(alpha), Mathf.Sin(theta) * Mathf.Sin(alpha), a * Mathf.Cos(theta)));
        T.SetRow(1, new Vector4(Mathf.Sin(theta), Mathf.Cos(theta) * Mathf.Cos(alpha), -Mathf.Cos(theta) * Mathf.Sin(alpha), a * Mathf.Sin(theta)));
        T.SetRow(2, new Vector4(0, Mathf.Sin(alpha), Mathf.Cos(alpha), d));
        T.SetRow(3, new Vector4(0, 0, 0, 1));
        return T;
    }

    /// <summary>
    /// Get Twist with Joint DH parameters
    /// </summary>
    public Matrix<float> GetTwistByDH(float[] theta, float[] d, float[] a, float[] alpha)
    {
        int n = theta.Length;
        Matrix<float> m = Matrix<float>.Build.Dense(6, n, 0);
        Matrix4x4 T = Matrix4x4.identity;
        for (int i = 0; i < n; i++)
        {
            T *= GetTransMatrix(theta[i], d[i], a[i], alpha[i]);
            Vector<float> Sw = Vector<float>.Build.Dense(3, 0);
            Vector<float> PO = Vector<float>.Build.Dense(3, 0);
            (Sw[0], Sw[1], Sw[2]) = ((int)T.m02, (int)T.m12, (int)T.m22);
            (PO[0], PO[1], PO[2]) = (T.m03, T.m13, T.m23);
            for (int j = 0; j < Sw.Count; j++) { if (Sw[j] == 1) PO[j] = 0; }
            var ss = SkewSymmetric(Sw);
            var Sv = -ss * PO;
            var S = Sw.ToColumnMatrix().Stack(Sv.ToColumnMatrix()).Column(0);
            m.SetColumn(i, S);
        }
        return m;
    }

    public Matrix<float> AxisAngle2Rot(Vector<float> S, float theta)
    {
        var m = SkewSymmetric(S);
        var I = Matrix<float>.Build.DenseIdentity(3);
        return I + Mathf.Sin(theta) * m + (1 - Mathf.Cos(theta)) * m * m;
    }

    public Matrix<float> Twist2HT(Vector<float> S, float theta)
    {
        Vector<float> w = Vector<float>.Build.Dense(3);
        Vector<float> v = Vector<float>.Build.Dense(3);
        (w[0], w[1], w[2]) = (S[0], S[1], S[2]);
        (v[0], v[1], v[2]) = (S[3], S[4], S[5]);

        var R = AxisAngle2Rot(w, theta);
        var I = Matrix<float>.Build.DenseIdentity(3);
        var skew_m = SkewSymmetric(w);
        var p = (I * theta + (1 - Mathf.Cos(theta)) * skew_m + (theta - Mathf.Sin(theta)) * skew_m * skew_m) * v;

        var m = p.ToColumnMatrix();
        var m1 = R.Append(m);

        var c = new[] { 0f, 0f, 0f, 1f };
        var v2 = Vector<float>.Build.DenseOfArray(c);
        var m2 = v2.ToRowMatrix();

        return m1.Stack(m2);
    }

    /// <summary>
    /// Get Adjoint Matrix
    /// </summary>
    public Vector<float> Adjoint(Vector<float> Sa, Matrix<float> T)
    {
        Matrix<float> R = Matrix<float>.Build.Dense(3, 3);
        Matrix<float> zeroM = Matrix<float>.Build.Dense(3, 3, 0);
        (R[0, 0], R[0, 1], R[0, 2]) = (T[0, 0], T[0, 1], T[0, 2]);
        (R[1, 0], R[1, 1], R[1, 2]) = (T[1, 0], T[1, 1], T[1, 2]);
        (R[2, 0], R[2, 1], R[2, 2]) = (T[2, 0], T[2, 1], T[2, 2]);

        Vector<float> p = Vector<float>.Build.Dense(3);
        (p[0], p[1], p[2]) = (T[3, 0], T[3, 1], T[3, 2]);
        var skew_p = SkewSymmetric(p);
        var m = skew_p * R;

        var m1 = R.Append(zeroM);
        var m2 = m.Append(R);
        var mm = m1.Stack(m2);

        return mm * Sa;
    }

    public Matrix<float> Fkine(Matrix<float> S, Matrix<float> M, float[] q, string frame = "space")
    {
        var col_num = S.ColumnCount;
        var T = Matrix<float>.Build.DenseIdentity(4);
        for (int i = 0; i < col_num; i++)
        {
            T *= Twist2HT(S.Column(i), q[i]);
        }
        if (frame == "space")
            T *= M;
        else
            T = M * T;
        return T;
    }
}
