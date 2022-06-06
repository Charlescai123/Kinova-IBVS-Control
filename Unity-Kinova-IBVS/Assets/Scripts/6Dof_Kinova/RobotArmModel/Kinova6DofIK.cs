using Kinova6Dof;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Complex;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Kinova6Dof
{
    public class Kinova6DofIK : MonoBehaviour
    {
        // Robot parameters
        public int numJoint = 6;
        [HideInInspector] public Kinova6DofFK fK;
        [HideInInspector] public RobotModelTool model;
        [HideInInspector] public Kinova6DofController controller;
        private Vector3[] jPositions;
        private Quaternion[] jRotations;

        private const float PI = Mathf.PI;
        private float[] joint_alpha = new float[] { PI, PI / 2, PI, PI / 2, PI / 2, PI / 2 };
        private float[] joint_a = new float[] { 0, 0, 0.41f, 0, 0, 0 };
        private float[] joint_d = new float[] { 0, -0.28481f, -0.005375f, -0.006375f, -0.31436f, 0 };
        private float[] joint_theta = new float[] { 0, 0, -PI / 2, -PI / 2, PI, PI };

        // Newton-Raphson parameters
        public int NR_Iterations = 1000;

        // CCD parameters
        public int CCD_Itreration = 20; // total iteration: n * (4*n)
        public float tolerancePosition = 0.01f; // m <==> 1cm
        public float toleranceRotation = 0.0872665f; // rad <==> 5°

        // Solution variables
        private float precise = 0.001f;
        private float[] jointAngles;
        private Vector3 targetPos;
        private Quaternion targetRot;
        private bool success;

        void Start()
        {
            // Robot parameters
            jointAngles = controller.homePositions;
        }

        void Update()
        {
            //for (int i = 0; i < numJoint; ++i)
            //    Debug.Log("Current Joint[" + i + "]:" + jointAngles[i].ToString("f4"));
            //var matrixA = DenseMatrix.OfArray(new[,] { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 } });
            var m = Matrix<float>.Build.DenseOfArray(new[,] { { 1.0f, 2.0f, 3.0f }, { 4.0f, 5.0f, 6.0f } });
            var v = Vector<float>.Build.DenseOfArray(new[] { 2f, 3f, 4f });
            var v1 = Vector<float>.Build.DenseOfArray(new[] { 7f, 8f, 9f });
            //var vector = new DenseVector(new[] { 1.0, 2.0, 3.0 });
            //var v = Vector<float>.Build.Dense(1, 3,{ 0,1,1});
            //var m = DenseMatrix.Build.Dense(3, 4, 1);
            var vv = v.ToColumnMatrix().Stack(v1.ToColumnMatrix()).Column(0);

            var mm = Matrix<float>.Build.Dense(2, 4, 0);
            var mmm = m.Append(mm);

            //Debug.LogError(mmm);

            //Debug.LogError(GetTwistByDH(joint_theta, joint_d, joint_a, joint_alpha));

            // Example //
            /*
            SetJointAngle(new float[] {0f, 0f, 0f, Mathf.PI/2, 0f, Mathf.PI/2, 0f});
            SetTarget(new Vector3(0.1f, 0.2f, 0.5f),
                      new Quaternion(0f, 0f, 1f, 0f));
            (float[] resultJointAngles, bool foundSolution) = CCD();

            string result = "";
            foreach (var a in resultJointAngles)
                result += a.ToString("0.000") + " ";
            Debug.Log(result);
            */
        }

        public void SetJointAngle(float[] currentJointAngles)
        {
            jointAngles = currentJointAngles;
        }
        public void SetTarget(Vector3 newTargetPosition, Quaternion newTargetRotation, bool fromRUF = true)
        {
            targetPos = newTargetPosition;
            targetRot = newTargetRotation;
            if (fromRUF)
            {
                targetPos = FromRUF(newTargetPosition);
                targetRot = FromRUF(newTargetRotation);
            }
        }

        public Matrix<float> JacobianAnalytical(Matrix<float> S, Matrix<float> M, float[] currQ)
        {
            var n = S.ColumnCount;
            var Js = Matrix<float>.Build.Dense(6, n, 0);

            Js.SetColumn(0, S.Column(0));
            var T = Matrix<float>.Build.DenseIdentity(4);

            for (int i = 1; i < n; i++)
            {
                var V = S.Column(i);
                var w = currQ[i];
                T *= model.Twist2HT(S.Column(i - 1), w);
                var A = model.Adjoint(V, T);
                Js.SetColumn(i, A);
            }

            var Jws = Js.SubMatrix(0, 3, 0, n);
            var Jvs = Js.SubMatrix(3, 3, 0, n);

            T = model.Fkine(S, M, currQ);

            Vector<float> p = Vector<float>.Build.Dense(3);
            (p[0], p[1], p[2]) = (T[3, 0], T[3, 1], T[3, 2]);

            Debug.Log("wtf:!!!!!!!!!!!!!!!" + model.SkewSymmetric(p) * Jws);
            Debug.Log("Jws:!!!!!!!!!!!!!!!" + Jws);
            var Ja = Jvs - model.SkewSymmetric(p) * Jws;
            return Ja;
        }

        /// <summary>
        /// Use Newton-Raphson Method to solve IK
        /// </summary>
        public (float[], bool) NewtonRaphson()
        {
            success = false;
            var (endPos, endRot) = fK.GetOneJointPose(numJoint);
            var M = Matrix<float>.Build.Dense(4, 4, 0);
            (M[0, 0], M[0, 1], M[0, 2], M[0, 3]) = (fK.M.m00, fK.M.m01, fK.M.m02, fK.M.m03);
            (M[1, 0], M[1, 1], M[1, 2], M[1, 3]) = (fK.M.m10, fK.M.m11, fK.M.m12, fK.M.m13);
            (M[2, 0], M[2, 1], M[2, 2], M[2, 3]) = (fK.M.m20, fK.M.m21, fK.M.m22, fK.M.m23);
            (M[3, 0], M[3, 1], M[3, 2], M[3, 3]) = (fK.M.m30, fK.M.m31, fK.M.m32, fK.M.m33);

            var S = model.GetTwistByDH(joint_theta, joint_d, joint_a, joint_alpha);

            var cnt = NR_Iterations;
            while ((targetPos - endPos).magnitude > precise && cnt > 0)
            {
                Debug.Log("cnmbgoudongxi:" + (targetPos - endPos).magnitude);
                var Ja = JacobianAnalytical(S, M, jointAngles);
                Debug.Log("Ja!!!!!!!!!!!!:" + Ja);

                // Newton-Raphson Method
                var JacoMat = Ja.PseudoInverse();
                //Jaco_Matrix = pinv(J_a);

                var v = Vector<float>.Build.Dense(3);
                var diff = targetPos - endPos;
                (v[0], v[1], v[2]) = (diff.x, diff.y, diff.z);
                Debug.Log("JacoMat!!!!!!!!!!!!:" + JacoMat);

                var deltaQ = JacoMat * v;
                Debug.Log("deltaQ:" + deltaQ);
                for (int i = 0; i < deltaQ.Count; i++)
                    jointAngles[i] += deltaQ[i];

                fK.UpdateAllHT(jointAngles);

                (endPos, endRot) = fK.GetOneJointPose(numJoint);
                cnt -= 1;
            }

            success = true;
            return (jointAngles, success);
        }

        /// <summary>
        /// CCD Algorithm to solve IK
        /// </summary>
        public (float[], bool) CCD()
        {
            fK.UpdateAllHT(jointAngles);
            success = false;

            // CCD Iteration
            for (int i = 0; i < CCD_Itreration; ++i)
            {
                // Check convergence
                var (endPosition, endRotation) = fK.GetOneJointPose(numJoint, true);
                if (IsPositionConverged(endPosition) && IsRotationConverged(endRotation))
                {
                    success = true;
                    break;
                }

                // Check position convergence
                (endPosition, endRotation) = fK.GetOneJointPose(numJoint, true);
                if (!IsPositionConverged(endPosition))
                {
                    // Minimize position error - backwards
                    for (int iJoint = numJoint - 1; iJoint >= 0; --iJoint)
                    {
                        (jPositions, jRotations) = fK.GetAllJointPose();
                        UpdateJointAngle(iJoint, true);
                        fK.UpdateOneHT(iJoint + 1, jointAngles[iJoint]);
                    }
                }

                // Check rotation convergence
                (endPosition, endRotation) = fK.GetOneJointPose(numJoint, true);
                if (!IsRotationConverged(endRotation))
                {
                    // Minimize rotation error - backwards
                    for (int iJoint = numJoint - 1; iJoint >= 0; --iJoint)
                    {
                        (jPositions, jRotations) = fK.GetAllJointPose();
                        UpdateJointAngle(iJoint, false);
                        fK.UpdateOneHT(iJoint + 1, jointAngles[iJoint]);
                    }
                }

                // Check position convergence again
                (endPosition, endRotation) = fK.GetOneJointPose(numJoint, true);
                if (!IsPositionConverged(endPosition))
                {
                    // Minimize position error - forwards
                    for (int iJoint = 0; iJoint < numJoint - 1; ++iJoint)
                    {
                        (jPositions, jRotations) = fK.GetAllJointPose();
                        UpdateJointAngle(iJoint, true);
                        fK.UpdateOneHT(iJoint + 1, jointAngles[iJoint]);
                    }
                }

                /*
                // Check rotation convergence
                (endPosition, endRotation) = fK.GetOneJointPose(numJoint);
                if (IsRotationConverged(endRotation))
                {
                    // Minimize rotation error - forwards
                    for (int iJoint = 0; iJoint < numJoint - 1; ++iJoint)
                    {
                        (jPositions, jRotations) = fK.GetAllJointPose();
                        UpdateJointAngle(iJoint, false);
                        fK.UpdateOneHT(iJoint + 1, jointAngles[iJoint]);
                    }
                }*/

            }
            var (endPositions, endRotations) = fK.GetOneJointPose(numJoint, true);
            Debug.Log("TCP Pos:" + endPositions.ToString("f4"));
            Debug.Log("TCP Rot:" + (endRotations.eulerAngles * Mathf.Deg2Rad).ToString("f4"));
            return (jointAngles, success);
        }

        /// <summary>
        /// Update Joint Angle
        /// </summary>
        private void UpdateJointAngle(int iJoint, bool PosFlag = true)
        {
            // Update position 
            Matrix4x4 H = Matrix4x4.TRS(Vector3.zero, jRotations[iJoint], Vector3.one);
            Vector3 jointZDirection = new Vector3(H[0, 2], H[1, 2], H[2, 2]);

            // Get rotation angle of next step
            float theta;
            if (PosFlag)
            {
                Vector3 endPosition = jPositions[numJoint];
                Vector3 jointPosition = jPositions[iJoint];

                // Unit vector from current joint to end effector
                Vector3 endVector = (endPosition - jointPosition).normalized;
                // Unit vector from current joint to target
                Vector3 targetVector = (targetPos - jointPosition).normalized;

                // Rotate current joint to match end effector vector to target vector
                float vectorAngle = Mathf.Clamp(Vector3.Dot(endVector, targetVector), -1, 1);
                theta = Mathf.Abs(Mathf.Acos(vectorAngle));
                Vector3 direction = Vector3.Cross(endVector, targetVector);

                // Map the desired angle to the angle of z aixs (for position)
                // ? Project normal axis to joint z axis ?
                theta = theta * Vector3.Dot(direction.normalized, jointZDirection.normalized);
            }
            else
            {
                Quaternion endRotation = jRotations[numJoint];
                // Rotate current joint to match end effector rotation to target rotation
                float errRotation = error(targetRot, endRotation);

                if (Mathf.Abs(Mathf.PI - errRotation) < 0.02f)
                    theta = 0.2f;
                else if (Mathf.Abs(errRotation) < 0.02f)
                    theta = 0;
                else
                {
                    // Rotate current joint to match end effector rotation to target rotation
                    Quaternion q = targetRot * Quaternion.Inverse(endRotation);
                    Vector3 direction = new Vector3(q.x, q.y, q.z) * q.w * 2 / Mathf.Sin(errRotation);

                    // Map the desired angle to the angle of z aixs (for rotation)
                    // ? Project normal axis to joint z axis ?
                    theta = errRotation * Vector3.Dot(direction.normalized, jointZDirection.normalized);
                }
            }
            jointAngles[iJoint] += theta;
            jointAngles[iJoint] = JointLimit(iJoint, jointAngles[iJoint]);
        }

        private float error(Vector3 p1, Vector3 p2)
        {
            return (p1 - p2).magnitude;
        }
        private float error(Quaternion q1, Quaternion q2)
        {
            Quaternion q = q1 * Quaternion.Inverse(q2);
            float theta = Mathf.Clamp(Mathf.Abs(q.w), -1, 1); // avoid overflow
            float errRotation = 2 * Mathf.Acos(theta);
            return errRotation;
        }
        private bool IsPositionConverged(Vector3 endPosition)
        {
            float errPosition = error(targetPos, endPosition);
            return errPosition < tolerancePosition;
        }
        private bool IsRotationConverged(Quaternion endRotation)
        {
            float errRotation = error(targetRot, endRotation);
            return errRotation < toleranceRotation;
        }

        /// <summary>
        /// Implement Joint Limits
        /// </summary>
        private float JointLimit(int iJoint, float angle)
        {
            // Apply joint limits
            float minAngle = fK.angleLowerLimits[iJoint];
            float maxAngle = fK.angleUpperLimits[iJoint];
            // If given joint limit
            if (minAngle != maxAngle)
                angle = Mathf.Clamp(angle, minAngle, maxAngle);
            // If no joint limit
            else
                angle = WrapToPi(angle);
            return angle;
        }

        /// <summary>
        /// Restraint angle between [-pi, pi]
        /// </summary>
        public static float WrapToPi(float angle)
        {
            // Wrap angle to [-pi, pi]
            float pi = Mathf.PI;
            angle = angle - 2 * pi * Mathf.Floor((angle + pi) / (2 * pi));
            return angle;
        }

        /// <summary>
        /// Convert from Unity coordinate to ROS (Position)
        /// </summary>
        private Vector3 FromRUF(Vector3 p)
        {
            return new Vector3(p.z, -p.x, p.y);
        }

        /// <summary>
        /// Convert from Unity coordinate to ROS (Orientation)
        /// </summary>
        private Quaternion FromRUF(Quaternion q)
        {
            return new Quaternion(q.z, -q.x, q.y, -q.w);
        }
    }

}
