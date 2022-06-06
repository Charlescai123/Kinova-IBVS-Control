using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Kinova6Dof
{
    public class Kinova6DofFK : MonoBehaviour
    {
        public int numJoint = 6;
        [HideInInspector] public RobotModelTool model;
        private const float PI = Mathf.PI;
        // DH parameters
        // params has length of numJoint + 1
        // world -> joint 1 -> ... -> end effector
        public float[] alpha = new float[] { PI, PI / 2, PI, PI / 2, PI / 2, PI / 2, PI };
        public float[] a = new float[] { 0, 0, 0.41f, 0, 0, 0, 0 };
        public float[] d = new float[] { 0, -0.28481f, -0.005375f, -0.006375f, -0.31436f, 0, -0.28743f };
        public float[] initialTheta = new float[] { 0, 0, -PI / 2, -PI / 2, PI, PI, PI };

        private float[] theta;
        public float[] angleLowerLimits = new float[] { 0, -2.25f, -2.58f, 0, -2.1f, 0 };
        public float[] angleUpperLimits = new float[] { 0, 2.25f, 2.58f, 0, 2.1f, 0 };

        // Homogeneous Transformation Matrix
        private Matrix4x4[] initH;
        private Matrix4x4[] HT;
        [HideInInspector] public Matrix4x4 M;

        // Joints position and rotation in real-time
        private Vector3[] jointPositions;
        private Quaternion[] jointRotations;

        void Start()
        {
            M = model.GetHomoMatrix(initialTheta, d, a, alpha);
            //Debug.LogError(M);

            theta = (float[])initialTheta.Clone();

            // Initialize homogeneous matrices
            initH = new Matrix4x4[numJoint + 1];
            for (int i = 0; i < numJoint + 1; ++i)
            {
                float ca = Mathf.Cos(alpha[i]);
                float sa = Mathf.Sin(alpha[i]);

                initH[i] = Matrix4x4.identity;
                initH[i].SetRow(0, new Vector4(1, -ca, sa, a[i]));
                initH[i].SetRow(1, new Vector4(1, ca, -sa, a[i]));
                initH[i].SetRow(2, new Vector4(0, sa, ca, d[i]));
                initH[i].SetRow(3, new Vector4(0, 0, 0, 1));
            }
            HT = (Matrix4x4[])initH.Clone();

            // Initialize joint positions and rotations
            jointPositions = new Vector3[numJoint + 1];
            jointRotations = new Quaternion[numJoint + 1];

            UpdateAllHT(new float[] { 0f, 0f, 0f, 0f, 0f, 0f, 0f });
        }

        /// <summary>
        /// Update HT Matrix with Joint Angle 
        /// </summary>
        public void UpdateOneHT(int i, float q)
        {
            /* Compute homography transformation matrix
               from joint i-1 to joint i 
            */
            // Update joint angle
            theta[i] = initialTheta[i] + q;
            // For computation
            float ct = Mathf.Cos(theta[i]);
            float st = Mathf.Sin(theta[i]);

            // Update Homogenous Transform Matrix
            HT[i] = initH[i];
            HT[i][0, 0] *= ct;
            HT[i][0, 1] *= st;
            HT[i][0, 2] *= st;
            HT[i][0, 3] *= ct;
            HT[i][1, 0] *= st;
            HT[i][1, 1] *= ct;
            HT[i][1, 2] *= ct;
            HT[i][1, 3] *= st;

            // Update joint positions and rotations
            UpdateAllJointPose();
        }


        /// <summary>
        /// Update All HT Matrices with Joint Angles
        /// </summary>
        public void UpdateAllHT(float[] jointAngles)
        {
            /* Compute homogeneous transformation matrices
               from joint i-1 to joint i for all i */
            UpdateOneHT(0, 0);
            for (int i = 0; i < numJoint; ++i)
            {
                UpdateOneHT(i + 1, jointAngles[i]);
            }

            // Update joint positions and rotations
            UpdateAllJointPose();
        }

        /// <summary>
        /// Update All Joint Configuration according to HT Matrix
        /// </summary>
        public void UpdateAllJointPose()
        {
            // Compute T from base to end effector
            Matrix4x4 T_End = Matrix4x4.identity;
            for (int i = 0; i < numJoint + 1; ++i)
            {
                T_End = T_End * HT[i];
                jointPositions[i] = new Vector3(T_End[0, 3], T_End[1, 3], T_End[2, 3]);
                jointRotations[i] = T_End.rotation;
                //Debug.Log("T[" + i + "] is:" + T_End);
            }
        }

        public (Vector3, Quaternion) GetOneJointPose(int i, bool toRUF = false)
        {
            // Unity coordinate
            if (toRUF)
                return (ToRUF(jointPositions[i]), ToRUF(jointRotations[i]));
            else
                return (jointPositions[i], jointRotations[i]);
        }
        public (Vector3[], Quaternion[]) GetAllJointPose(bool toRUF = false)
        {
            // Unity coordinate
            if (toRUF)
            {
                Vector3[] positionsRUF = new Vector3[numJoint + 1];
                Quaternion[] rotationsRUF = new Quaternion[numJoint + 1];
                for (int i = 0; i < numJoint + 1; ++i)
                {
                    positionsRUF[i] = ToRUF(jointPositions[i]);
                    rotationsRUF[i] = ToRUF(jointRotations[i]);
                }
                return (positionsRUF, rotationsRUF);
            }
            else
                return (jointPositions, jointRotations);
        }

        /// <summary>
        /// Convert from ROS coordinate to Unity (Position)
        /// </summary>
        private Vector3 ToRUF(Vector3 p)
        {
            return new Vector3(-p.y, p.z, p.x);
        }

        /// <summary>
        /// Convert from ROS coordinate to Unity (Orientation)
        /// </summary>
        private Quaternion ToRUF(Quaternion q)
        {
            return new Quaternion(-q.y, q.z, q.x, -q.w);
        }

    }
}

