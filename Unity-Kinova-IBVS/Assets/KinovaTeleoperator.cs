using Kinova6Dof;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;


public class KinovaTeleoperator : MonoBehaviour
{
    public static readonly string[] KinovaLinks =
       { "Base_Link/Shoulder_Link", "/Bicep_Link", "/ForeArm_Link", "/SphericalWrist1_Link", "/SphericalWrist2_Link", "/Bracelet_Link" };

    // Hardcoded variables
    const int k_NumRobotJoints = 6;

    // Articulation Bodies
    ArticulationBody[] MAJointArtiBodies;
    public ArticulationGripperController ManipulatorGripper;

    public bool TeleoperationOn = false;
    public bool IsGripperOpen = true;

    private Vector3 lastPosition;
    private Vector3 lastPositionForCalc;
    private bool origin = true;
    private Vector3 startOrigin;

    // Robot
    public GameObject jointRoot;
    public GameObject endEffector;

    private float[] currJointAngles;
    private int jointLength;

    // Controllers
    public Kinova6DofController MAJointController;
    public Kinova6DofIK iK;

    // Position Control
    private Vector3 deltaPosition;
    private Quaternion prevRotation;

    // Rotation Control
    private Vector3 deltaRotation;
    private Vector3 prevPosition;

    // Variables for Color Highlight
    private int index = 0;
    private int preIndex = 0;
    private Color[] preColor;
    [Tooltip("Color to highlight the currently selected join")]
    public Color highLightColor = new Color(1.0f, 0, 0, 1.0f);

    /***************** Get the current configuration for joints *****************/
    enum KinovaArm
    {
        ManipulatorArm,
        CameraArm
    }

    // Start is called before the first frame update
    void Start()
    {
        jointLength = iK.numJoint;
        currJointAngles = new float[jointLength];

        // Get joints
        MAJointArtiBodies = jointRoot.GetComponentsInChildren<ArticulationBody>();
        MAJointArtiBodies = MAJointArtiBodies.Where(joint => joint.jointType != ArticulationJointType.FixedJoint).ToArray();
        MAJointArtiBodies = MAJointArtiBodies.Take(k_NumRobotJoints).ToArray();

        StoreJointColors(MAJointArtiBodies, preIndex);
        Highlight(MAJointArtiBodies, index);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.N))
        {
            MAJointController.SetJointVelocity(MAJointArtiBodies[0], 0, 1f);
            //Debug.LogError("shit:" + MAJointArtiBodies[1].);
            //var effect = wtf.stiffness*()-wtf.damping*()
        }
    }

    // Fixed Update
    void FixedUpdate()
    {
        deltaPosition = Vector3.zero;
        float delta = 0.01f; // 0.05 sample size

        // x -> unity coordinate
        if (Input.GetKey(KeyCode.J))
        {
            deltaPosition += new Vector3(0, 0, delta);
        }
        else if (Input.GetKey(KeyCode.L))
        {
            deltaPosition += new Vector3(0, 0, -delta);
        }
        // y -> unity coordinate
        if (Input.GetKey(KeyCode.K))
        {
            deltaPosition += new Vector3(-delta, 0, 0);
        }
        else if (Input.GetKey(KeyCode.I))
        {
            deltaPosition += new Vector3(delta, 0, 0);
        }
        if (deltaPosition != Vector3.zero)
        {
            prevRotation = endEffector.transform.rotation;
            prevPosition = endEffector.transform.position;
            float[] newJoints = IKSolver(MAJointArtiBodies, deltaPosition, Vector3.zero);
            for (int i = 0; i < newJoints.Length; ++i)
            {
                Debug.Log("newJoint[" + i + "]:" + newJoints[i] * Mathf.Rad2Deg);
                //jointController.SetJointTarget(MAJointArtiBodies[i], newJoints[i]);
            }
            //AllJointGo(MAJointArtiBodies, newJoints);
        }
    }

    private void LateUpdate()
    {
        // Home Kinova Manipulator Arm
        if (Input.GetKeyDown(KeyCode.M))
        {
            MAJointController.HomeJoints();
        }

        //lastPosition = pos;

        // Enter the Teleoperation Status
        if (OVRInput.GetDown(OVRInput.Button.SecondaryHandTrigger))
        {
            //lastPositionForCalc = pos;
            TeleoperationOn = true;
        }

        // Exit the Teleoperation Status
        if (OVRInput.GetUp(OVRInput.Button.SecondaryHandTrigger))
        {
            TeleoperationOn = false;
            //MAJointController.StopAllJoint(MAJointArtiBodies);
        }

        if (TeleoperationOn)
        {
            KinovaJointPosControl();
        }
    }
    public void AllJointGo(ArticulationBody[] articulationbody, float[] q)
    {
        for (int i = 0; i < q.Length; i++)
        {
            var drive = articulationbody[i].xDrive;
            q[i] *= Mathf.Rad2Deg;
            drive.target = q[i];
            articulationbody[i].xDrive = drive;
        }
    }

    private void KinovaJointPosControl()
    {
        var vel = OVRInput.GetLocalControllerVelocity(OVRInput.Controller.RHand);
        var pos = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RHand);
        // Index to next Joint
        if (OVRInput.GetDown(OVRInput.Button.One))
        {
            index = ((index + 1) + k_NumRobotJoints) % k_NumRobotJoints;
            Highlight(MAJointArtiBodies, index);
        }
        // Index to last Joint
        else if (OVRInput.GetDown(OVRInput.Button.Two))
        {
            index = ((index - 1) + k_NumRobotJoints) % k_NumRobotJoints;
            Highlight(MAJointArtiBodies, index);
        }

        var deltaPos = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick);

        // Open/Close Gripper
        if (OVRInput.GetDown(OVRInput.Button.SecondaryIndexTrigger))
        {
            if (!IsGripperOpen)
                ManipulatorGripper.OpenGrippers();
            else
                ManipulatorGripper.CloseGrippers();
            // Reverse the Gripper State Flag
            IsGripperOpen ^= true;
        }

        Debug.Log("Teleoperating!!!");
        var p = MAJointArtiBodies[index].xDrive.target * Mathf.Deg2Rad;
        p += deltaPos.x * 0.005f;

        MAJointController.SetJointTarget(MAJointArtiBodies[index], p);
        //var jointVel = Math.Sqrt(Math.Pow(vel.x, 2) + Math.Pow(vel.y, 2) + Math.Pow(vel.z, 2));

        //Debug.Log("vel of xDrive is:" + jointVel.ToString("f4"));
        //jointXDrive.targetVelocity = (float)jointVel * 1000;

        Debug.Log("joint velocity is:" + MAJointArtiBodies[index].velocity.ToString("f4"));
        //lastPositionForCalc = pos;
    }

    private void KinovaJointVelControl()
    {
        var vel = OVRInput.GetLocalControllerVelocity(OVRInput.Controller.RHand);
        var pos = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RHand);
        // Index to next Joint
        if (OVRInput.GetDown(OVRInput.Button.One))
        {
            index = ((index + 1) + k_NumRobotJoints) % k_NumRobotJoints;
            Highlight(MAJointArtiBodies, index);
        }
        // Index to last Joint
        else if (OVRInput.GetDown(OVRInput.Button.Two))
        {
            index = ((index - 1) + k_NumRobotJoints) % k_NumRobotJoints;
            Highlight(MAJointArtiBodies, index);
        }

        vel.x = (float)Math.Round(vel.x, 1);
        vel.y = (float)Math.Round(vel.y, 1);
        vel.z = (float)Math.Round(vel.z, 1);

        var velocity = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick);

        // Open/Close Gripper
        if (OVRInput.GetDown(OVRInput.Button.SecondaryIndexTrigger))
        {
            if (!IsGripperOpen)
                ManipulatorGripper.OpenGrippers();
            else
                ManipulatorGripper.CloseGrippers();
            // Reverse the Gripper State Flag
            IsGripperOpen ^= true;
        }

        var deltaPos = pos - lastPositionForCalc;
        Debug.Log("Teleoperating!!!");
        var velo = MAJointArtiBodies[index].xDrive.targetVelocity * Mathf.Deg2Rad;
        velo += velocity.x * 0.005f;
        Debug.Log("velo is:!!!" + velo);
        Debug.Log("velocity x is:!!!" + velocity.x);
        MAJointController.SetJointVelocity(MAJointArtiBodies[index], index, velo);
        //var jointVel = Math.Sqrt(Math.Pow(vel.x, 2) + Math.Pow(vel.y, 2) + Math.Pow(vel.z, 2));
        var jointVel = vel.y;
        //Debug.Log("vel of xDrive is:" + jointVel.ToString("f4"));
        //jointXDrive.targetVelocity = (float)jointVel * 1000;

        Debug.Log("joint velocity is:" + MAJointArtiBodies[index].velocity.ToString("f4"));
        //lastPositionForCalc = pos;
    }

    private float[] IKSolver(ArticulationBody[] articulationChain, Vector3 deltaPosition, Vector3 deltaRotation)
    {
        // Target position and rotation   
        Vector3 position = jointRoot.transform.InverseTransformPoint(
                              endEffector.transform.position) + deltaPosition;
        Vector3 rotation = (Quaternion.Inverse(jointRoot.transform.rotation) *
                            endEffector.transform.rotation).eulerAngles + deltaRotation;

        if (deltaPosition == Vector3.zero)
        {
            position = jointRoot.transform.InverseTransformPoint(prevPosition);
        }
        if (deltaRotation == Vector3.zero)
        {
            rotation = (Quaternion.Inverse(jointRoot.transform.rotation) * prevRotation).eulerAngles;
        }

        // Solve IK
        // get current joints
        for (int i = 0; i < jointLength; ++i)
        {
            // currJointAngles[i] = articulationChain[i].jointPosition[0]; 
            // Use drive target instead of exact joint position
            // to avoid unintended oscillation due to controller's static error
            currJointAngles[i] = articulationChain[i].xDrive.target * Mathf.Deg2Rad;
            Debug.Log("wtf[" + i + "]:" + currJointAngles[i]);
        }

        iK.SetJointAngle(currJointAngles);

        // set target
        iK.SetTarget(position, Quaternion.Euler(rotation.x, rotation.y, rotation.z));
        Debug.Log("cnmsbdongxi:" + position);

        // solve
        (float[] resultJointAngles, bool foundSolution) = iK.CCD();

        return resultJointAngles;
    }


    /***************************** Robot Arm Highlight *****************************/
    private void Highlight(ArticulationBody[] body, int selectedIndex)
    {
        if (selectedIndex < 0 || selectedIndex >= body.Length)
            return;

        // reset colors for the previously selected joint
        ResetJointColors(body, preIndex);

        // store colors for the current selected joint
        StoreJointColors(body, selectedIndex);

        // DisplaySelectedJoint(selectedIndex);
        Renderer[] rendererList = body[selectedIndex].transform.Find("Visuals").GetComponentsInChildren<Renderer>();

        // set the color of the selected join meshes to the highlight color
        foreach (var mesh in rendererList)
        {
            MaterialExtensions.SetMaterialColor(mesh.material, highLightColor);
        }
        preIndex = selectedIndex;
    }

    private void StoreJointColors(ArticulationBody[] body, int index)
    {
        Renderer[] materialLists = body[index].transform.Find("Visuals").GetComponentsInChildren<Renderer>();
        preColor = new Color[materialLists.Length];
        for (int counter = 0; counter < materialLists.Length; counter++)
        {
            preColor[counter] = MaterialExtensions.GetMaterialColor(materialLists[counter]);
        }
    }
    private void ResetJointColors(ArticulationBody[] body, int index)
    {
        Renderer[] previousRendererList = body[index].transform.Find("Visuals").GetComponentsInChildren<Renderer>();
        for (int counter = 0; counter < previousRendererList.Length; counter++)
        {
            MaterialExtensions.SetMaterialColor(previousRendererList[counter].material, preColor[counter]);
        }
    }

}
