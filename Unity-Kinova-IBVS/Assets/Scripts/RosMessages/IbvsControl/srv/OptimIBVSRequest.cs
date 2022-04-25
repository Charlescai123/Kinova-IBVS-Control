//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.IbvsControl
{
    [Serializable]
    public class OptimIBVSRequest : Message
    {
        public const string k_RosMessageName = "ibvs_control/OptimIBVS";
        public override string RosMessageName => k_RosMessageName;

        public Std.Float64Msg[] qc;
        //  6
        public Std.Float64Msg[] dqc;
        //  6
        public Std.Float64Msg[] qt;
        //  6
        public Std.Float64Msg[] dqt;
        //  6
        public PointOnImgMsg pt;
        //  tool com
        public PointOnImgMsg pg;
        //  goal com
        public PointOnImgMsg po;
        //  occlusion com
        public Std.Float64Msg area_g;
        public Std.Float64Msg area_o;
        public PointOnImgMsg[] pg_seg;
        //  4 points of goal segmentation
        public PointOnImgMsg[] po_seg;
        //  4 points of obstacle segmentation

        public OptimIBVSRequest()
        {
            this.qc = new Std.Float64Msg[0];
            this.dqc = new Std.Float64Msg[0];
            this.qt = new Std.Float64Msg[0];
            this.dqt = new Std.Float64Msg[0];
            this.pt = new PointOnImgMsg();
            this.pg = new PointOnImgMsg();
            this.po = new PointOnImgMsg();
            this.area_g = new Std.Float64Msg();
            this.area_o = new Std.Float64Msg();
            this.pg_seg = new PointOnImgMsg[0];
            this.po_seg = new PointOnImgMsg[0];
        }

        public OptimIBVSRequest(Std.Float64Msg[] qc, Std.Float64Msg[] dqc, Std.Float64Msg[] qt, Std.Float64Msg[] dqt, PointOnImgMsg pt, PointOnImgMsg pg, PointOnImgMsg po, Std.Float64Msg area_g, Std.Float64Msg area_o, PointOnImgMsg[] pg_seg, PointOnImgMsg[] po_seg)
        {
            this.qc = qc;
            this.dqc = dqc;
            this.qt = qt;
            this.dqt = dqt;
            this.pt = pt;
            this.pg = pg;
            this.po = po;
            this.area_g = area_g;
            this.area_o = area_o;
            this.pg_seg = pg_seg;
            this.po_seg = po_seg;
        }

        public static OptimIBVSRequest Deserialize(MessageDeserializer deserializer) => new OptimIBVSRequest(deserializer);

        private OptimIBVSRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.qc, Std.Float64Msg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.dqc, Std.Float64Msg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.qt, Std.Float64Msg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.dqt, Std.Float64Msg.Deserialize, deserializer.ReadLength());
            this.pt = PointOnImgMsg.Deserialize(deserializer);
            this.pg = PointOnImgMsg.Deserialize(deserializer);
            this.po = PointOnImgMsg.Deserialize(deserializer);
            this.area_g = Std.Float64Msg.Deserialize(deserializer);
            this.area_o = Std.Float64Msg.Deserialize(deserializer);
            deserializer.Read(out this.pg_seg, PointOnImgMsg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.po_seg, PointOnImgMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.qc);
            serializer.Write(this.qc);
            serializer.WriteLength(this.dqc);
            serializer.Write(this.dqc);
            serializer.WriteLength(this.qt);
            serializer.Write(this.qt);
            serializer.WriteLength(this.dqt);
            serializer.Write(this.dqt);
            serializer.Write(this.pt);
            serializer.Write(this.pg);
            serializer.Write(this.po);
            serializer.Write(this.area_g);
            serializer.Write(this.area_o);
            serializer.WriteLength(this.pg_seg);
            serializer.Write(this.pg_seg);
            serializer.WriteLength(this.po_seg);
            serializer.Write(this.po_seg);
        }

        public override string ToString()
        {
            return "OptimIBVSRequest: " +
            "\nqc: " + System.String.Join(", ", qc.ToList()) +
            "\ndqc: " + System.String.Join(", ", dqc.ToList()) +
            "\nqt: " + System.String.Join(", ", qt.ToList()) +
            "\ndqt: " + System.String.Join(", ", dqt.ToList()) +
            "\npt: " + pt.ToString() +
            "\npg: " + pg.ToString() +
            "\npo: " + po.ToString() +
            "\narea_g: " + area_g.ToString() +
            "\narea_o: " + area_o.ToString() +
            "\npg_seg: " + System.String.Join(", ", pg_seg.ToList()) +
            "\npo_seg: " + System.String.Join(", ", po_seg.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
