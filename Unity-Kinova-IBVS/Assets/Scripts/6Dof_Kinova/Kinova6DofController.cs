using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

namespace Kinova6Dof
{
    public class Kinova6DofController : MonoBehaviour
    {
        // Use Gravity and Inertia
        public bool useGravity;
        public bool useInertia;

        // Robot Properties
        public GameObject JointRoot;
        public int numJoint = 6;

        // Joint Normal Exec Speed
        private const float jointNormalSpeed = 1f;

        // Articulation Bodies
        public float[] homePositions = { 0f, 0f, 0f, 0f, 0f, 0f };
        public float[] jointVelocityLimits = { 1.3963f, 1.3963f, 1.3963f, 1.2218f, 1.2218f, 1.2218f };
        private ArticulationBody[] articulationChain;

        //private bool test = true;
        private bool[] homeFinFlag;

        //Joint xDrive Parameters
        public float stiffness = 10000f;
        public float damping = 100f;
        public float forceLimit = 1000f;

        void Start()
        {
            // Initalize Joint Parameters
            KinovaJointParamInit(JointRoot);

            homeFinFlag = new bool[homePositions.Length];
            for (int i = 0; i < homeFinFlag.Length; i++)
                homeFinFlag[i] = false;

            // Get joints
            articulationChain = JointRoot.GetComponentsInChildren<ArticulationBody>();
            articulationChain = articulationChain.Where(joint => joint.jointType != ArticulationJointType.FixedJoint).ToArray();
            articulationChain = articulationChain.Take(numJoint).ToArray();

            // Home Configuration
            HomeJoints();
        }

        void Update()
        {

            //if (Input.GetKeyDown(KeyCode.N))
            //{
            //    test ^= true;
            //}
            //if (test)
            //{
            //    var wtf = articulationChain[0].xDrive;
            //    wtf.target += 1.2f;
            //    articulationChain[0].xDrive = wtf;
            //}
            //var wtf1 = articulationChain[0].xDrive;
            //Debug.Log("target:" + wtf1.target);

        }

        public void HomeJoints()
        {
            for (int i = 0; i < homeFinFlag.Length; i++)
                homeFinFlag[i] = false;
            StartCoroutine(HomeJointsCoroutine());
        }
        private IEnumerator HomeJointsCoroutine()
        {
            yield return new WaitUntil(() => HomeAndCheck() == true);
        }
        private bool HomeAndCheck()
        {
            float precise = 0.0001f;
            for (int i = 0; i < homePositions.Length; ++i)
            {
                // prevent conversion error deg->rad
                float current = articulationChain[i].xDrive.target * Mathf.Deg2Rad;
                var error = Mathf.Abs(current - homePositions[i]);
                if (error > precise)
                    SetJointTargetStep(articulationChain[i], homePositions[i]);
                else
                    homeFinFlag[i] = true;
            }
            // Check if all errors are under range
            for (int i = 0; i < homeFinFlag.Length; i++)
            {
                if (homeFinFlag[i] == false)
                {
                    Debug.Log("Fails to home Joint Configuration of " + i);
                    return false;
                }
            }
            return true;
        }

        public void SetJointTarget(ArticulationBody joint, float target)
        {
            if (float.IsNaN(target))
                return;
            target = target * Mathf.Rad2Deg;

            // Get drive
            ArticulationDrive drive = joint.xDrive;

            // Joint limit
            if (joint.twistLock == ArticulationDofLock.LimitedMotion)
            {
                if (target > drive.upperLimit)
                    target = drive.upperLimit;
                else if (target < drive.lowerLimit)
                    target = drive.lowerLimit;
            }

            // Set target
            drive.target = target;
            joint.xDrive = drive;
        }

        public void SetJointTargetStep(ArticulationBody joint, float target, float speed = jointNormalSpeed)
        {
            // Return if target input is NaN
            if (float.IsNaN(target))
            {
                Debug.Log("Target value is NaN!");
                return;
            }

            // Rad to Deg
            target = target * Mathf.Rad2Deg;

            // Get drive
            ArticulationDrive drive = joint.xDrive;
            float currentTarget = drive.target;

            // Speed limit
            float deltaPosition = speed * Mathf.Rad2Deg * Time.fixedDeltaTime;
            if (Mathf.Abs(currentTarget - target) > deltaPosition)
                target = currentTarget + deltaPosition * Mathf.Sign(target - currentTarget);

            // Joint limit
            if (joint.twistLock == ArticulationDofLock.LimitedMotion)
            {
                if (target > drive.upperLimit)
                    target = drive.upperLimit;
                else if (target < drive.lowerLimit)
                    target = drive.lowerLimit;
            }

            // Set target
            drive.target = target;
            drive.targetVelocity = 0;
            joint.xDrive = drive;
        }

        public void SetJointVelocity(ArticulationBody joint, int index, float velocity)
        {
            // Restrain Joint Velocity to [-velocityLimit, velocityLimit]
            if (Mathf.Abs(velocity) > jointVelocityLimits[index])
                velocity = jointVelocityLimits[index];
            velocity *= Mathf.Rad2Deg;      // Rad to Deg
            var drive = joint.xDrive;
            drive.stiffness = 0;
            drive.targetVelocity = velocity;
            joint.xDrive = drive;
        }

        public void SetJointVelocityStep(ArticulationBody joint, float speed = jointNormalSpeed)
        {
            // Get drive
            ArticulationDrive drive = joint.xDrive;
            float currentTarget = drive.target;

            // Speed limit
            float deltaPosition = speed * Mathf.Rad2Deg * Time.fixedDeltaTime;
            float target = currentTarget + deltaPosition;

            // Joint limit
            if (joint.twistLock == ArticulationDofLock.LimitedMotion)
            {
                if (target > drive.upperLimit)
                    target = drive.upperLimit;
                else if (target < drive.lowerLimit)
                    target = drive.lowerLimit;
            }

            // Set target
            drive.target = target;
            joint.xDrive = drive;
        }

        public void StopAllJoint(ArticulationBody[] joint)
        {
            for(int i = 0; i < joint.Length; i++)
            {
                var drive = joint[i].xDrive;
                float currPosition = joint[i].jointPosition[0] * Mathf.Rad2Deg;
                drive.target = currPosition;
                drive.targetVelocity = 0;
                drive.stiffness = stiffness;        // Set value from 0 to original settings
                joint[i].xDrive = drive;               
            }
        }

        public void StopOneJoint(ArticulationBody joint)
        {
            float currPosition = joint.jointPosition[0] * Mathf.Rad2Deg;
            ArticulationDrive drive = joint.xDrive;
            if (Mathf.Abs(drive.target - currPosition) > 1)
            {
                drive.target = currPosition;
                joint.xDrive = drive;
            }
        }

        // Get All Joints Target
        public float[] GetCurrentJointTargets()
        {
            float[] targets = new float[articulationChain.Length];
            for (int i = 0; i < articulationChain.Length; ++i)
            {
                targets[i] = articulationChain[i].xDrive.target;
            }
            targets = targets.Select(r => r * Mathf.Deg2Rad).ToArray();
            return targets;
        }

        // Init All Kinova Joints xDrives
        public void KinovaJointParamInit(GameObject root, bool assignToAllChildren = true, int robotChainLength = 0)
        {
            // Get non-fixed joints
            var articulationbody = root.GetComponentsInChildren<ArticulationBody>();
            var inertia_group = root.GetComponentsInChildren<UrdfInertial>();

            // Whether use gravity
            if (!useGravity)
                for (int i = 0; i < articulationbody.Length; ++i)
                    articulationbody[i].useGravity = false;

            // Whether use inertia
            if (!useInertia)
                for (int i = 0; i < inertia_group.Length; ++i)
                    inertia_group[i].useUrdfData = false;

            // Get joints except for fixed ones
            articulationbody = articulationbody.Where(joint => joint.jointType != ArticulationJointType.FixedJoint).ToArray();

            // Joint length to assign
            int assignLength = articulationbody.Length;
            if (!assignToAllChildren)
                assignLength = robotChainLength;

            // Setting stiffness, damping and force limit
            int defDyanmicVal = 100;
            for (int i = 0; i < assignLength; ++i)
            {
                ArticulationBody joint = articulationbody[i];
                ArticulationDrive drive = joint.xDrive;

                joint.jointFriction = defDyanmicVal;
                joint.angularDamping = defDyanmicVal;

                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = forceLimit;

                joint.xDrive = drive;
            }
        }

        public void Jacobian(Vector3 endEffectPos, Vector3 endEffectRot)
        {
            var w_s1 = new Vector3(0, 0, -1);
            var w_s2 = new Vector3(0, 1, 0);
            var w_s3 = new Vector3(0, -1, 0);
            var w_s4 = new Vector3(0, 0, -1);
            var w_s5 = new Vector3(0, -1, 0);
            var w_s6 = new Vector3(0, 0, -1);

            var p_s1 = new Vector3(0, 0, 0);
            var p_s2 = new Vector3(0, 0, 0.2848f);
            var p_s3 = new Vector3(0, 0, 0.6948f);
            var p_s4 = new Vector3(0, 0.001f, 0);
            var p_s5 = new Vector3(0, 0, 0.3804f);
            var p_s6 = new Vector3(0, 0.001f, 0);

            

        }
    }
}
