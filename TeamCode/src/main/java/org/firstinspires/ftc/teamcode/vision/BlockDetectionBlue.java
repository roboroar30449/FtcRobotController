package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.*;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class BlockDetectionBlue {

    private OpenCvCamera webcam;
    private EdgePnPPipeline pipeline;

    // Initialize the vision system with hardware map and camera name
    public void init(HardwareMap hardwareMap, String cameraName) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, cameraName),
                cameraMonitorViewId);

        pipeline = new EdgePnPPipeline();
        webcam.setPipeline(pipeline);
    }

    // Open camera and start streaming
    public void startStreaming() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // You may want to log or handle error here
            }
        });
    }

    // Stop camera streaming and close device
    public void stopStreaming() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    // Accessor methods for vision results
    public double getDeltaX() {
        return pipeline.deltaX;
    }

    public double getDeltaY() {
        return pipeline.deltaY;
    }

    public double getHeadingDeg() {
        return pipeline.headingDeg;
    }

    // Internal vision pipeline class
    static class EdgePnPPipeline extends OpenCvPipeline {
        Mat cameraMatrix;
        MatOfDouble distCoeffs;

        public double deltaX = 0;
        public double deltaY = 0;
        public double headingDeg = 0;

        private final Scalar lowerBlue = new Scalar(90, 100, 100);
        private final Scalar upperBlue = new Scalar(130, 255, 255);

        private Point objectCenter = null;
        private Point[] contourPoints = null;

        public EdgePnPPipeline() {
            cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            double[] camData = {
                    242.38165191, 0, 314.83509711,
                    0, 242.45074231, 223.11661867,
                    0, 0, 1
            };
            cameraMatrix.put(0, 0, camData);

            distCoeffs = new MatOfDouble(-0.0132293, -0.01237019, -0.00077488, -0.00088637, -0.0010935);
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Mat mask = new Mat();
            Core.inRange(hsv, lowerBlue, upperBlue, mask);

            Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)));
            Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5)));

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                MatOfPoint largest = contours.get(0);
                double maxArea = Imgproc.contourArea(largest);
                for (MatOfPoint c : contours) {
                    double area = Imgproc.contourArea(c);
                    if (area > maxArea) {
                        largest = c;
                        maxArea = area;
                    }
                }

                MatOfPoint2f largest2f = new MatOfPoint2f(largest.toArray());
                RotatedRect box = Imgproc.minAreaRect(largest2f);

                Point[] pts = new Point[4];
                box.points(pts);
                contourPoints = pts;

                objectCenter = new Point(box.center.x, box.center.y);

                MatOfPoint3f objectPoints = new MatOfPoint3f(
                        new Point3(-1, -1, 0),
                        new Point3(1, -1, 0),
                        new Point3(1, 1, 0),
                        new Point3(-1, 1, 0)
                );

                MatOfPoint2f imagePoints = new MatOfPoint2f(pts);

                Mat rvec = new Mat();
                Mat tvec = new Mat();
                boolean success = Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

                if (success) {
                    deltaX = tvec.get(0,0)[0];
                    deltaY = tvec.get(1,0)[0];
                    headingDeg = Math.toDegrees(rvec.get(1,0)[0]);
                }
            } else {
                objectCenter = null;
                contourPoints = null;
                deltaX = 0;
                deltaY = 0;
                headingDeg = 0;
            }

            if (contourPoints != null) {
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, contourPoints[i], contourPoints[(i+1) % 4], new Scalar(0,255,0), 3);
                }
            }

            if (objectCenter != null) {
                Imgproc.circle(input, objectCenter, 7, new Scalar(0,255,0), -1);
            }

            Imgproc.circle(input, new Point(cameraMatrix.get(0, 2)[0], cameraMatrix.get(1, 2)[0]), 7, new Scalar(0,255,0), -1);

            return input;
        }
    }
}
