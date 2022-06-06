using Kinova6Dof;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using Unity.Robotics;
using UnityEngine;

public enum ControlMode { Position = 0, Velocity = 1 };
public enum ControlType { Joint = 0, TCPPos = 1, TCPRot = 2 };

public class KeyboardControl : MonoBehaviour
{
    // Control modes
    [HideInInspector] public ControlMode controlMode;
    public ControlType controlType;

    // Robot
    public GameObject jointRoot;
    public GameObject endEffector;
    private ArticulationBody[] articulationChain;
    private float[] currJointAngles;
    private int jointLength;

    // Controllers
    [HideInInspector] public GripperController gripperController;
    [HideInInspector] public Kinova6DofController jointController;
    [HideInInspector] public Kinova6DofIK iK;

    public float jointSpeed = 0.002f;
    public float linearSpeed = 0.06f;
    public float angularSpeed = 0.15f;

    // Joint Control
    private int prevIndex;
    private int selectedIndex;
    private Color[] prevColor;
    private Color highLightColor;
    private float currPos;

    // Position Control
    private Vector3 deltaPosition;
    private Quaternion prevRotation;

    // Rotation Control
    private Vector3 deltaRotation;
    private Vector3 prevPosition;

    [GUITarget(2)]    // Display 3
    public void OnGUI()
    {
        GUIStyle style = GUI.skin.GetStyle("Label");
        style.alignment = TextAnchor.UpperLeft;
        var selectedJoint = articulationChain[selectedIndex].name + " (" + selectedIndex + ")";
        GUI.Label(new Rect(Screen.width / 2 - 200, 10, 640, 20), "Press left/right arrow keys to switch control type: " + controlType.ToString() + " Control", style);
        if (controlType == ControlType.Joint)
            GUI.Label(new Rect(Screen.width / 2 - 200, 30, 400, 20), "Press A/D keys to switch joint: " + selectedJoint, style);
        else
        {
            style.alignment = TextAnchor.UpperCenter;
            GUI.Label(new Rect(Screen.width / 2 - 200, 30, 400, 20), "Press A, D, W, S, Q, E keys to move", style);
        }
    }

    void Start()
    {
        // Get joints
        articulationChain = jointRoot.GetComponentsInChildren<ArticulationBody>();
        articulationChain = articulationChain.Where(joint => joint.jointType
                                                    != ArticulationJointType.FixedJoint).ToArray();
        jointLength = iK.numJoint;
        currJointAngles = new float[jointLength];

        // Default control mode - Joint Control
        controlMode = ControlMode.Position;
        controlType = ControlType.Joint;

        // Initialize Joint Control
        prevIndex = 0;
        StoreJointColors(articulationChain, prevIndex);
        selectedIndex = 0;
        highLightColor = new Color(1.0f, 0, 0, 1.0f);
        Highlight(articulationChain, selectedIndex);

        // Initialize Position and Rotation Control
        deltaRotation = Vector3.zero;
        deltaPosition = Vector3.zero;
    }

    void Update()
    {
        // Press H to home joint configuration
        if (Input.GetKeyDown(KeyCode.H))
            jointController.HomeJoints();

        // Open/Close Gripper
        if (Input.GetKeyDown(KeyCode.G))
        {
            if (gripperController.gripperStatus == GripperStatus.Close)
            {
                gripperController.OpenGrippers();
            }
            else if (gripperController.gripperStatus == GripperStatus.Open)
            {
                gripperController.CloseGrippers();
            }
        }

        // Switch control type
        if (Input.GetKeyDown(KeyCode.LeftArrow))
        {
            SwitchType((int)controlType - 1);
        }
        else if (Input.GetKeyDown(KeyCode.RightArrow))
        {
            SwitchType((int)controlType + 1);
        }

        // Switch control mode
        if (Input.GetKeyDown(KeyCode.UpArrow))
        {
            var numMode = Enum.GetNames(typeof(ControlMode)).Length;
            controlMode = (ControlMode)((((int)controlMode + 1) + numMode) % numMode);
        }
        else if (Input.GetKeyDown(KeyCode.DownArrow))
        {
            var numMode = Enum.GetNames(typeof(ControlMode)).Length;
            controlMode = (ControlMode)((((int)controlMode - 1) + numMode) % numMode);
        }

        // Joint control
        if (controlType == ControlType.Joint)
        {
            // switch joint
            if (Input.GetKeyDown(KeyCode.A))
            {
                selectedIndex = ((selectedIndex - 1) + iK.numJoint) % iK.numJoint;
                Highlight(articulationChain, selectedIndex);
            }
            else if (Input.GetKeyDown(KeyCode.D))
            {
                selectedIndex = ((selectedIndex + 1) + iK.numJoint) % iK.numJoint;
                Highlight(articulationChain, selectedIndex);
            }

            // control to move
            if (Input.GetKey(KeyCode.W))
            {
                currPos += jointSpeed;
                jointController.SetJointTarget(articulationChain[selectedIndex], currPos);
            }
            else if (Input.GetKey(KeyCode.S))
            {
                currPos -= jointSpeed;
                jointController.SetJointTarget(articulationChain[selectedIndex], currPos);
            }
            else
            {
                currPos = articulationChain[selectedIndex].xDrive.target * Mathf.Deg2Rad;
            }
        }

        // TCP Position control
        else if (controlType == ControlType.TCPPos)
        {
            deltaPosition = Vector3.zero;
            float delta = linearSpeed; // 0.05 sample size

            //deltaPosition += new Vector3(0, 0, delta);
            // x -> unity coordinate
            if (Input.GetKey(KeyCode.A))
            {
                deltaPosition += new Vector3(0, 0, -delta);
            }
            else if (Input.GetKey(KeyCode.D))
            {
                deltaPosition += new Vector3(0, 0, delta);
            }
            // y -> unity coordinate
            if (Input.GetKey(KeyCode.W))
            {
                deltaPosition += new Vector3(-delta, 0, 0);
            }
            else if (Input.GetKey(KeyCode.S))
            {
                deltaPosition += new Vector3(delta, 0, 0);
            }
            // z -> unity coordinate
            if (Input.GetKey(KeyCode.Q))
            {
                deltaPosition += new Vector3(0, -delta, 0);
            }
            else if (Input.GetKey(KeyCode.E))
            {
                deltaPosition += new Vector3(0, delta, 0);
            }

            // Solve Position IK
            if (deltaPosition != Vector3.zero)
            {
                var res = SolveIK(deltaPosition, deltaRotation);
                for (int i = 0; i < res.Length; i++)
                    Debug.Log("Current Joint[" + i + "]:" + Mathf.Rad2Deg * res[i]);
                MoveAllJoints(res);
            }
        }

        // TCP Rotation control
        else if (controlType == ControlType.TCPRot)
        {
            deltaRotation = Vector3.zero;
            float delta = angularSpeed * Mathf.Rad2Deg;

            // x -> unity coordinate
            if (Input.GetKey(KeyCode.A))
            {
                deltaRotation += new Vector3(0, 0, -delta);
            }
            else if (Input.GetKey(KeyCode.D))
            {
                deltaRotation += new Vector3(0, 0, delta);
            }
            // y -> unity coordinate
            if (Input.GetKey(KeyCode.W))
            {
                deltaRotation += new Vector3(delta, 0, 0);
            }
            else if (Input.GetKey(KeyCode.S))
            {
                deltaRotation += new Vector3(-delta, 0, 0);
            }
            // z -> unity coordinate
            if (Input.GetKey(KeyCode.Q))
            {
                deltaRotation += new Vector3(0, -delta, 0);
            }
            else if (Input.GetKey(KeyCode.E))
            {
                deltaRotation += new Vector3(0, delta, 0);
            }

            // Solve Rotation IK
            if (deltaRotation != Vector3.zero)
            {
                var res = SolveIK(deltaPosition, deltaRotation);
                for (int i = 0; i < res.Length; i++)
                    Debug.Log("Current Joint[" + i + "]:" + Mathf.Rad2Deg * res[i]);
                MoveAllJoints(res);
            }
        }
    }

    private void SwitchType(int ctrlType)
    {
        // Switch to type ctrlType
        int numType = Enum.GetNames(typeof(ControlType)).Length;
        int typeIndex = (ctrlType + numType) % numType;
        controlType = (ControlType)typeIndex;

        // Change color for entering and leaving joint control mode
        if (controlType == ControlType.Joint)
        {
            Highlight(articulationChain, selectedIndex);
        }
        else if (controlType == ControlType.TCPPos)
        {
            ResetJointColors(articulationChain, selectedIndex);
            prevRotation = endEffector.transform.rotation;
        }
        else if (controlType == ControlType.TCPRot)
        {
            ResetJointColors(articulationChain, selectedIndex);
            prevPosition = endEffector.transform.position;
        }

        // Set delta to 0
        if (controlType == ControlType.TCPPos)
            deltaRotation = Vector3.zero;
        else if (controlType == ControlType.TCPRot)
            deltaPosition = Vector3.zero;
        Debug.Log("Switch to type: " + controlType);
    }

    private float[] SolveIK(Vector3 deltaPos, Vector3 deltaRot)
    {
        // Target position and rotation
        Vector3 position = jointRoot.transform.InverseTransformPoint(
                              endEffector.transform.position) + deltaPos;
        Vector3 rotation = (Quaternion.Inverse(jointRoot.transform.rotation) *
                            endEffector.transform.rotation).eulerAngles + deltaRot;
        Debug.Log("Target Pos:" + position.ToString("f4"));
        Debug.Log("Target Rot:" + (rotation * Mathf.Deg2Rad).ToString("f4"));

        if (deltaPos == Vector3.zero)
        {
            position = jointRoot.transform.InverseTransformPoint(prevPosition);
        }

        if (deltaRot == Vector3.zero)
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
        }

        iK.SetJointAngle(currJointAngles);

        // set target
        iK.SetTarget(position, Quaternion.Euler(rotation.x, rotation.y, rotation.z));

        // solve
        (float[] resultJointAngles, bool foundSolution) = iK.CCD();
        //(float[] resultJointAngles, bool foundSolution) = iK.NewtonRaphson();

        for (int i = 0; i < resultJointAngles.Length; ++i)
            resultJointAngles[i] = Kinova6DofIK.WrapToPi(resultJointAngles[i]);

        return resultJointAngles;
    }

    public void MoveAllJoints(float[] q)
    {
        for (int i = 0; i < q.Length; ++i)
        {
            jointController.SetJointTarget(articulationChain[i], q[i]);
        }
    }


    /***************************** Robot Arm Joint Highlight *****************************/
    private void Highlight(ArticulationBody[] body, int selectedIndex)
    {
        if (selectedIndex < 0 || selectedIndex >= body.Length)
            return;

        // reset colors for the previously selected joint
        ResetJointColors(body, prevIndex);

        // store colors for the current selected joint
        StoreJointColors(body, selectedIndex);

        // DisplaySelectedJoint(selectedIndex);
        Renderer[] rendererList = body[selectedIndex].transform.Find("Visuals").GetComponentsInChildren<Renderer>();

        // set the color of the selected join meshes to the highlight color
        foreach (var mesh in rendererList)
        {
            MaterialExtensions.SetMaterialColor(mesh.material, highLightColor);
        }
        prevIndex = selectedIndex;
    }

    private void StoreJointColors(ArticulationBody[] body, int index)
    {
        Renderer[] materialLists = body[index].transform.Find("Visuals").GetComponentsInChildren<Renderer>();
        prevColor = new Color[materialLists.Length];
        for (int counter = 0; counter < materialLists.Length; counter++)
        {
            prevColor[counter] = MaterialExtensions.GetMaterialColor(materialLists[counter]);
        }
    }
    private void ResetJointColors(ArticulationBody[] body, int index)
    {
        Renderer[] previousRendererList = body[index].transform.Find("Visuals").GetComponentsInChildren<Renderer>();
        for (int counter = 0; counter < previousRendererList.Length; counter++)
        {
            MaterialExtensions.SetMaterialColor(previousRendererList[counter].material, prevColor[counter]);
        }
    }
}
