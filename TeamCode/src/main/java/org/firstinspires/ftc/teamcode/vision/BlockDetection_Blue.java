package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlockDetection_Blue {
    RobotHardware robot;
    Telemetry telemetry;
    public EdgePnPPipeline pipeline;
    public static double deltaX = 0;
    public static double deltaY = 0;
    public static double headingDeg = 0;

    public BlockDetection_Blue(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public static class EdgePnPPipeline extends OpenCvPipeline {
        // camera calibration
        Mat cameraMatrix;
        MatOfDouble distCoeffs;
        private final Scalar lowerBlue = new Scalar(90, 100, 100);
        private final Scalar upperBlue = new Scalar(130, 255, 255);
        private Point objectCenter = null;
        private Point[] contourPoints = null;

        public EdgePnPPipeline() {
            cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            double[] camData = {242.38, 0, 314.83, 0, 242.45, 223.11, 0, 0, 1};
            cameraMatrix.put(0, 0, camData);
            distCoeffs = new MatOfDouble(-0.0132, -0.01237, -0.000774, -0.000886, -0.0010935);
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
                contourPoints = new Point[4];
                box.points(contourPoints);
                objectCenter = box.center;

                MatOfPoint3f objectPoints = new MatOfPoint3f(
                        new Point3(-1, -1, 0),
                        new Point3(1, -1, 0),
                        new Point3(1, 1, 0),
                        new Point3(-1, 1, 0)
                );
                MatOfPoint2f imagePoints = new MatOfPoint2f(contourPoints);
                Mat rvec = new Mat();
                Mat tvec = new Mat();
                boolean success = Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
                if (success) {
                    deltaX = tvec.get(0,0)[0];
                    deltaY = tvec.get(1,0)[0];
                    Mat R = new Mat();
                    Calib3d.Rodrigues(rvec, R);
                    double yaw = Math.atan2(R.get(1,0)[0], R.get(0,0)[0]);
                    headingDeg = Math.toDegrees(yaw);
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

    public void allVisionTelemetry() {
        telemetry.addData("ΔX (in)", deltaX);
        telemetry.addData("ΔY (in)", deltaY);
        telemetry.addData("Heading (deg)", headingDeg);
        telemetry.addData("Detection State", (deltaX != 0) ? "Detected" : "Detecting");
        telemetry.update();
    }
}