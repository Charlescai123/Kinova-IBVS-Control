using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Kinova6Dof
{
    public class CameraCaptor : MonoBehaviour
    {
        // Object Capture Property
        public GameObject goal;
        public GameObject TCP;
        public GameObject obstacle;
        public Camera VisualServoCam;

        // Extract obstacle upper side center point and 4 corner points
        [HideInInspector] public Transform obsCenterPoint;
        [HideInInspector] public Transform[] obsCornerPoints;

        // Extract Goal upper side center point and 4 corner points
        [HideInInspector] public Transform goalCenterPoint;
        [HideInInspector] public Transform[] goalCornerPoints;

        // 2D Image Pixel of last frame
        private Vector2 lastGoalImgPixel;        // Goal
        private Vector2 lastTCPImgPixel;         // TCP
        private Vector2 lastObsImgPixel;         // Obstacle (Upper side center point)
        private Vector2[] lastObsSegImgPixels;   // Obstacle Segment (Cube)
        private Vector2[] lastGoalSegImgPixels;  // Goal Segment (VC Box)

        void Awake()
        {
            // Initial Array
            obsCornerPoints = new Transform[4];
            goalCornerPoints = new Transform[4];
            lastObsSegImgPixels = new Vector2[4];
            lastGoalSegImgPixels = new Vector2[4];

            // Extract obstacle and goal information
            obsCenterPoint = obstacle.transform.Find("Upper Center Point");
            goalCenterPoint = goal.transform.Find("Upper Center Point");
            var name = "Corner Point";
            for (int i = 0; i < 4; i++)
            {
                var order = i + 1;
                var str = name + order.ToString();
                obsCornerPoints[i] = obstacle.transform.Find(str);
                goalCornerPoints[i] = goal.transform.Find(str);
            }

            // First get image pixel
            lastGoalImgPixel = Get2DImgPixel(VisualServoCam, goal.transform);
            lastTCPImgPixel = Get2DImgPixel(VisualServoCam, TCP.transform);
            lastObsImgPixel = Get2DImgPixel(VisualServoCam, obsCenterPoint);
            for (int i = 0; i < 4; i++)
            {
                lastObsSegImgPixels[i] = Get2DImgPixel(VisualServoCam, obsCornerPoints[i]);
                lastGoalSegImgPixels[i] = Get2DImgPixel(VisualServoCam, goalCornerPoints[i]);
            }

        }

        // Update is called once per frame
        void Update()
        {
            // Press Space to take screenshot
            if (Input.GetKeyDown(KeyCode.Space))
            {
                TakeScreenShot(VisualServoCam, new Rect(0, 0, Screen.width, Screen.height));
            }
            // Press C to display Visual Servoing Camera Parameters
            if (Input.GetKeyDown(KeyCode.C)) { GetCameraParam(); }

            // Mouse right click to obtain pixel info on image
            if (Input.GetMouseButtonDown(1))
            {
                Debug.Log("Mouse Input Point Pixel on Image: Width:" + Input.mousePosition.x + ", Height:" + Input.mousePosition.y);
            }

            var vel = Get2DImgPixelVel(VisualServoCam, TCP.transform);
        }

        private void LateUpdate()
        {
            UpdatePixelPerFrame();      // Update Img Pixel per frame
        }

        /// <summary>
        /// Update image pixel information
        /// </summary>
        void UpdatePixelPerFrame()
        {
            lastGoalImgPixel = Get2DImgPixel(VisualServoCam, goal.transform);
            lastTCPImgPixel = Get2DImgPixel(VisualServoCam, TCP.transform);
            lastObsImgPixel = Get2DImgPixel(VisualServoCam, obsCenterPoint);
            for (int i = 0; i < 4; i++)
            {
                lastObsSegImgPixels[i] = Get2DImgPixel(VisualServoCam, obsCornerPoints[i]);
                lastGoalSegImgPixels[i] = Get2DImgPixel(VisualServoCam, goalCornerPoints[i]);
            }
        }
        /// <summary>
        /// Take Camera Screenshot
        /// </summary>
        /// <returns>The screenshot</returns>
        /// <param name="camera">Camera</param>
        /// <param name="rect">Rect.Areas to be taken for screenshot</param>
        public Texture2D TakeScreenShot(Camera camera, Rect rect, string filepath = "/Snapshot/Screenshot")
        {
            // A RenderTexture instance
            RenderTexture rt = new RenderTexture((int)rect.width, (int)rect.height, 0);

            // Render the camera
            camera.targetTexture = rt;
            camera.Render();

            // Activate rt, and read pixel from it
            RenderTexture.active = rt;
            Texture2D screenShot = new Texture2D((int)rect.width, (int)rect.height, TextureFormat.RGB24, false);
            screenShot.ReadPixels(rect, 0, 0);
            screenShot.Apply();

            // Reset camera param
            camera.targetTexture = null;
            RenderTexture.active = null; // JC: added to avoid errors
            GameObject.Destroy(rt);

            // Encode to PNG bytes
            byte[] PNGbytes = screenShot.EncodeToPNG();

            // PNG Name: Screenshot-[Time].png
            filepath = Application.dataPath + filepath;
            string dirPath = System.IO.Path.GetDirectoryName(filepath);
            filepath = filepath + " " + System.DateTime.Now.ToString("yyyy-MM-dd HHmmss") + ".png";

            // If directory name doesn't exist, create one
            if (!Directory.Exists(dirPath))
                Directory.CreateDirectory(dirPath);

            System.IO.File.WriteAllBytes(filepath, PNGbytes);
            Debug.Log(string.Format("A screenshot has been saved: {0}", filepath));
            return screenShot;
        }

        /// <summary>
        /// Print information about Visual Servoing Camera (Intrinsic/Extrinsic Parameters)
        /// </summary>
        public void GetCameraParam()
        {
            Debug.Log("Shift:" + VisualServoCam.lensShift.x);
            Debug.Log("Shift:" + VisualServoCam.lensShift.y);
            Debug.Log("VS Camera Pixel Rect is:" + VisualServoCam.pixelRect);
            Debug.Log("VS Camera Projection Matrix (Intrinsic) is:" + VisualServoCam.projectionMatrix.ToString("f4"));
            //Debug.Log("VS Camera Projection Matrix (Extrinsic) is:" + VisualServoCam.worldToCameraMatrix.ToString("f4"));
        }

        /// <summary>
        /// Used for mapping 3D object transform to 2D image transform
        /// (Pixel coordinate origin is set at top left of the whole image)
        /// </summary>
        public Vector3 Get2DImgPixel(Camera cam, Transform worldtf)
        {
            var worldPos = worldtf.position;
            var worldVector = new Vector4(worldPos.x, worldPos.y, worldPos.z, 1);
            var camViewPos = cam.worldToCameraMatrix * worldVector;
            Vector4 projPos = cam.projectionMatrix * camViewPos;
            Vector3 ndcPos = new Vector3(projPos.x / projPos.w, projPos.y / projPos.w, projPos.z / projPos.w);
            Vector3 viewportPos = new Vector3(ndcPos.x * 0.5f + 0.5f, ndcPos.y * 0.5f + 0.5f, -camViewPos.z);
            var u = viewportPos.x * cam.pixelWidth;
            var v = cam.pixelHeight - viewportPos.y * cam.pixelHeight;  // Origin set to top-left corner
            var depth = viewportPos.z;
            return new Vector3(u, v, depth);
        }

        /// <summary>
        /// Used for obtaining 3D object velocity on 2D image
        /// (Pixel coordinate origin is set at left above of the whole image)
        /// </summary>
        public Vector2 Get2DImgPixelVel(Camera cam, Transform worldtf)
        {
            Vector2 vel = new Vector2(0, 0);

            // Goal
            if (worldtf == this.goal.transform)
            {
                Vector2 currPixel = Get2DImgPixel(cam, this.goal.transform);
                vel = (currPixel - this.lastGoalImgPixel) / Time.deltaTime;
            }

            // TCP
            else if (worldtf == this.TCP.transform)
            {
                Vector2 currPixel = Get2DImgPixel(cam, this.TCP.transform);
                vel = (currPixel - this.lastTCPImgPixel) / Time.deltaTime;
            }

            // Obstacle Upper Bound Center Point
            else if (worldtf == this.obsCenterPoint)
            {
                Vector2 currPixel = Get2DImgPixel(cam, this.obsCenterPoint);
                vel = (currPixel - this.lastObsImgPixel) / Time.deltaTime;
            }

            // Goal Upper Bound Center Point
            else if (worldtf == this.goalCenterPoint)
            {
                Vector2 currPixel = Get2DImgPixel(cam, this.goalCenterPoint);
                vel = (currPixel - this.lastGoalImgPixel) / Time.deltaTime;
            }

            else
            {
                for (int i = 0; i < 4; i++)
                {
                    // Obstacle Corner Point
                    if (worldtf == this.obsCornerPoints[i].transform)
                    {
                        Vector2 currPixel = Get2DImgPixel(cam, this.obsCornerPoints[i]);
                        vel = (currPixel - this.lastObsSegImgPixels[i]) / Time.deltaTime;
                        break;
                    }

                    // Goal Corner Point
                    if (worldtf == this.goalCornerPoints[i].transform)
                    {
                        Vector2 currPixel = Get2DImgPixel(cam, this.goalCornerPoints[i]);
                        vel = (currPixel - this.lastGoalSegImgPixels[i]) / Time.deltaTime;
                        break;
                    }

                }
            }
            return vel;
        }
    }
}

