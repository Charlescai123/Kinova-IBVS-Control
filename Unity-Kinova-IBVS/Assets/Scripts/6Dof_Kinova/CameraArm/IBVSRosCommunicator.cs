using RosMessageTypes.Moveit;
using RosMessageTypes.IbvsControl;
using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Kinova6Dof;
using UnityEngine;

public class IBVSRosCommunicator : MonoBehaviour
{
    // Variables required for ROS communication
    [SerializeField]
    public string RosServiceName;
    public int ROSUpdateRate;

    // Robot Properties
    private int numJoint = 6;
    public GameObject MAJointRoot;
    public GameObject CAJointRoot;
    public Kinova6DofController CAController;
    private ArticulationBody[] MAJointArtiBodies;
    private ArticulationBody[] CAJointArtiBodies;

    // Camera Captor
    public CameraCaptor camCaptor;

    // ROS Connector
    ROSConnection m_Ros;

    // ROS Variables
    private bool ROSOnFlag = false;
    private bool ROSReceivedFlag = false;
    private bool StateOFlag = false;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<OptimIBVSRequest, OptimIBVSResponse>(RosServiceName);

        // Get MA joints
        MAJointArtiBodies = MAJointRoot.GetComponentsInChildren<ArticulationBody>();
        MAJointArtiBodies = MAJointArtiBodies.Where(joint => joint.jointType != ArticulationJointType.FixedJoint).ToArray();
        MAJointArtiBodies = MAJointArtiBodies.Take(numJoint).ToArray();

        // Get CA joints
        CAJointArtiBodies = CAJointRoot.GetComponentsInChildren<ArticulationBody>();
        CAJointArtiBodies = CAJointArtiBodies.Where(joint => joint.jointType != ArticulationJointType.FixedJoint).ToArray();
        CAJointArtiBodies = CAJointArtiBodies.Take(numJoint).ToArray();

        // Call Function to update Data
        InvokeRepeating("IBVSSrvCall", 1f, 1 / ROSUpdateRate);
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.B))
        {
            ROSOnFlag = false;
            CAController.StopAllJoint(CAJointArtiBodies);
        }
        if (Input.GetKeyDown(KeyCode.P))
        {
            ROSOnFlag ^= true;
        }
        // Send State O Flag
        if (Input.GetKeyDown(KeyCode.V))
        {
            StateOFlag ^= true;
        }

        if (ROSOnFlag)
        {
            IBVSSrvCall();
        }

    }

    // Send IBVS Service Call using ROS
    public void IBVSSrvCall()
    {
        // Return if ROS is turned off
        if (!ROSOnFlag)
            return;

        var request = new OptimIBVSRequest();

        // Position and Velocity of Camera Arm and Manipulation Arm
        request.qc = GetJointConfiguration(CAJointArtiBodies);
        request.qt = GetJointConfiguration(MAJointArtiBodies);
        request.dqc = GetJointVelocity(CAJointArtiBodies);
        request.dqt = GetJointVelocity(MAJointArtiBodies);

        // Image Point Info
        request.pt = GetImgPointInfo(camCaptor.TCP.transform);          // TCP
        request.pg = GetImgPointInfo(camCaptor.goal.transform);         // Goal

        request.po = GetImgPointInfo(camCaptor.obsCenterPoint);         // Obstacle Center
        request.po_seg = GetImgSegmentInfo(camCaptor.obstacle.transform);  // Obstacle Segment
        request.pg_seg = GetImgSegmentInfo(camCaptor.goal.transform);    // Goal Segment

        // Obstacle Area
        //request.area_o = GetQuadAreaSize(camCaptor.obsCornerPoints);
        Float64Msg wtf = new Float64Msg();
        if (StateOFlag)
        {
            wtf.data = 1;
            request.area_o = wtf;
        }
        else
        {
            wtf.data = 0;
            request.area_o = wtf;
        }
        Debug.Log("State O Flag is:" + wtf.data);

        // Goal Area
        request.area_g = GetQuadAreaSize(camCaptor.goalCornerPoints);

        // Send ROS Service Message
        if (!ROSReceivedFlag)
        {
            m_Ros.SendServiceMessage<OptimIBVSResponse>(RosServiceName, request, ServiceResponse);
            ROSReceivedFlag = true;
        }
        Debug.Log("I'm now publishing!");
    }

    // Process received IVBS Response
    private void ServiceResponse(OptimIBVSResponse response)
    {
        Debug.Log("Received!!!");
        for (int i = 0; i < response.dqc_next.Length; i++)
        {
            //Debug.Log(response.dqc_next[i]);
            CAController.SetJointVelocity(CAJointArtiBodies[i], i, (float)response.dqc_next[i].data);
        }
        ROSReceivedFlag = false;
    }

    // Get Joint Configuration
    private Float64Msg[] GetJointConfiguration(ArticulationBody[] articulationbody)
    {
        Float64Msg[] msg = new Float64Msg[articulationbody.Length];
        for (int i = 0; i < articulationbody.Length; i++)
        {
            msg[i] = new Float64Msg();
            msg[i].data = articulationbody[i].jointPosition[0];
        }
        return msg;
    }

    // Get Joint Velocity
    private Float64Msg[] GetJointVelocity(ArticulationBody[] articulationbody)
    {
        Float64Msg[] msg = new Float64Msg[articulationbody.Length];
        for (int i = 0; i < articulationbody.Length; i++)
        {
            msg[i] = new Float64Msg();
            msg[i].data = articulationbody[i].jointVelocity[0];
        }
        return msg;
    }

    // Get Point Info on Image (Position, Velocity, Depth)
    private PointOnImgMsg GetImgPointInfo(Transform world_tf)
    {
        // Position (u,v)
        var pixel = camCaptor.Get2DImgPixel(camCaptor.VisualServoCam, world_tf);
        Float64Msg[] pos = new Float64Msg[2];
        pos[0] = new Float64Msg();
        pos[1] = new Float64Msg();
        pos[0].data = pixel.x;
        pos[1].data = pixel.y;

        // Velocity (vel_u, vel_v)
        var pixelVel = camCaptor.Get2DImgPixelVel(camCaptor.VisualServoCam, world_tf);
        Float64Msg[] vel = new Float64Msg[2];
        vel[0] = new Float64Msg();
        vel[1] = new Float64Msg();
        vel[0].data = pixelVel.x;
        vel[1].data = pixelVel.y;

        // Depth (z)
        Float64Msg depth = new Float64Msg();
        depth.data = pixel.z;

        PointOnImgMsg msg = new PointOnImgMsg(pos, vel, depth);
        return msg;
    }

    // Get Obstacle/Goal Segment Information (4 Corner Point)
    private PointOnImgMsg[] GetImgSegmentInfo(Transform obj)
    {
        PointOnImgMsg[] msgs = new PointOnImgMsg[4];

        Transform centerPoint = obj.Find("Upper Center Point");
        Transform cornerPoint1 = obj.Find("Corner Point1");
        Transform cornerPoint2 = obj.Find("Corner Point2");
        Transform cornerPoint3 = obj.Find("Corner Point3");
        Transform cornerPoint4 = obj.Find("Corner Point4");

        msgs[0] = new PointOnImgMsg();
        msgs[1] = new PointOnImgMsg();
        msgs[2] = new PointOnImgMsg();
        msgs[3] = new PointOnImgMsg();

        msgs[0] = GetImgPointInfo(cornerPoint1);
        msgs[1] = GetImgPointInfo(cornerPoint2);
        msgs[2] = GetImgPointInfo(cornerPoint3);
        msgs[3] = GetImgPointInfo(cornerPoint4);

        return msgs;

    }

    private Float64Msg GetQuadAreaSize(Transform[] tfs)
    {
        Float64Msg areaSize = new Float64Msg();
        Vector2[] points = new Vector2[tfs.Length];

        for (int i = 0; i < tfs.Length; i++)
            points[i] = camCaptor.Get2DImgPixel(camCaptor.VisualServoCam, tfs[i]);

        float diagonal = CalcDistance(points[0], points[2]);
        float edge_01 = CalcDistance(points[0], points[1]);
        float edge_12 = CalcDistance(points[1], points[2]);
        float edge_23 = CalcDistance(points[2], points[3]);
        float edge_30 = CalcDistance(points[3], points[0]);

        var s1 = CalcTriangeArea(edge_01, edge_12, diagonal);
        var s2 = CalcTriangeArea(edge_23, edge_30, diagonal);

        areaSize.data = s1 + s2;

        return areaSize;
    }

    private float CalcDistance(Vector2 p1, Vector2 p2)
    {
        return Mathf.Sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    public float CalcTriangeArea(float a, float b, float c)
    {
        float p = (a + b + c) / 2; ;
        return (Mathf.Sqrt(p * (p - a) * (p - b) * (p - c)));
    }
}
