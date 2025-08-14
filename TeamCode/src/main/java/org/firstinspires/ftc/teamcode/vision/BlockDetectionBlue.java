package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

public class BlockDetectionBlue {

    OpenCvCamera webcam;
    BlockDetection.EdgePnPPipeline pipeline;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private boolean isStreaming = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new BlockDetection.EdgePnPPipeline();
        webcam.setPipeline(pipeline);
    }

    public void startStreaming() {
        if (webcam != null && !isStreaming) {
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    isStreaming = true;
                }

                @Override
                public void onError(int errorCode) {
                    if (telemetry != null) {
                        telemetry.addData("Camera open error", errorCode);
                        telemetry.update();
                    }
                }
            });
        }
    }

    public void stopStreaming() {
        if (webcam != null && isStreaming) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
            isStreaming = false;
        }
    }

    public boolean isStreaming() {
        return isStreaming;
    }

    public double getDeltaX() {
        return (pipeline != null) ? pipeline.deltaX : 0;
    }

    public double getDeltaY() {
        return (pipeline != null) ? pipeline.deltaY : 0;
    }

    public double getHeadingDeg() {
        return (pipeline != null) ? pipeline.headingDeg : 0;
    }

    static class EdgePnPPipeline extends OpenCvPipeline {
        Mat cameraMatrix;
        MatOfDouble distCoeffs;

        // Public outputs
        public double deltaX = 0;
        public double deltaY = 0;
        public double headingDeg = 0;

        // HSV blue color range for detection
        private final Scalar lowerBlue = new Scalar(90, 100, 100);
        private final Scalar upperBlue = new Scalar(130, 255, 255);

        private Point objectCenter = null;
        private Point[] contourPoints = null;

        public EdgePnPPipeline() {
            // Camera Matrix
            cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            double[] camData = {
                    242.38165191, 0, 314.83509711,
                    0, 242.45074231, 223.11661867,
                    0, 0, 1
            };
            cameraMatrix.put(0, 0, camData);

            // Distortion Coefficients
            distCoeffs = new MatOfDouble(-0.0132293, -0.01237019, -0.00077488, -0.00088637, -0.0010935);
        }

        @Override
        public Mat processFrame(Mat input) {
            // Convert input to HSV color space to detect blue
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Mat mask = new Mat();
            Core.inRange(hsv, lowerBlue, upperBlue, mask);

            // Morphology to reduce noise
            Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
            Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

            // Find contours on mask
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                // Find largest contour
                MatOfPoint largest = contours.get(0);
                double maxArea = Imgproc.contourArea(largest);
                for (MatOfPoint c : contours) {
                    double area = Imgproc.contourArea(c);
                    if (area > maxArea) {
                        largest = c;
                        maxArea = area;
                    }
                }

                // Approximate contour polygon and bounding box
                MatOfPoint2f largest2f = new MatOfPoint2f(largest.toArray());
                RotatedRect box = Imgproc.minAreaRect(largest2f);

                // Store contour points for drawing edges
                Point[] pts = new Point[4];
                box.points(pts);
                contourPoints = pts;

                // Calculate center of detected object
                objectCenter = new Point(box.center.x, box.center.y);

                // Calculate the longest edge of the bounding box
                double d01 = distance(pts[0], pts[1]);
                double d12 = distance(pts[1], pts[2]);
                double d23 = distance(pts[2], pts[3]);
                double d30 = distance(pts[3], pts[0]);

                Point edgeStart = pts[0], edgeEnd = pts[1];
                double maxLen = d01;

                if (d12 > maxLen) {
                    edgeStart = pts[1];
                    edgeEnd = pts[2];
                    maxLen = d12;
                }
                if (d23 > maxLen) {
                    edgeStart = pts[2];
                    edgeEnd = pts[3];
                    maxLen = d23;
                }
                if (d30 > maxLen) {
                    edgeStart = pts[3];
                    edgeEnd = pts[0];
                    maxLen = d30;
                }

                // Calculate angle of the longest edge with respect to vertical axis (y-axis)
                double dx = edgeEnd.x - edgeStart.x;
                double dy = edgeEnd.y - edgeStart.y;

                // Calculate angle relative to vertical axis
                double angleRad = Math.atan2(dx, dy); // Invert dx, dy to make the angle relative to the vertical axis
                double angleDeg = Math.toDegrees(angleRad);

                // Directly use this angle as heading
                headingDeg = angleDeg;

                // Optionally compute pose for position only
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
                    deltaX = tvec.get(0, 0)[0];
                    deltaY = tvec.get(1, 0)[0];
                }
            } else {
                objectCenter = null;
                contourPoints = null;
                deltaX = 0;
                deltaY = 0;
                headingDeg = 0;
            }

            // Draw detected contour edges in green
            if (contourPoints != null) {
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, contourPoints[i], contourPoints[(i + 1) % 4], new Scalar(0, 255, 0), 3);
                }
            }

            // Draw green dot at detected object center
            if (objectCenter != null) {
                Imgproc.circle(input, objectCenter, 7, new Scalar(0, 255, 0), -1);
            }

            // Draw green dot at center of the camera frame
            Imgproc.circle(input, new Point(cameraMatrix.get(0, 2)[0], cameraMatrix.get(1, 2)[0]), 7, new Scalar(0, 255, 0), -1);

            return input;
        }

        // Utility function to compute Euclidean distance
        private double distance(Point p1, Point p2) {
            return Math.hypot(p2.x - p1.x, p2.y - p1.y);
        }
    }
}