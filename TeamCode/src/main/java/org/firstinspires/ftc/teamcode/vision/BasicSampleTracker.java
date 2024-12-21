package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class BasicSampleTracker extends LinearOpMode {

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    //public static final double focalLength = 728;

    @Override
    public void runOpMode(){
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        waitForStart();

        while (opModeIsActive()) {

        }
        controlHubCam.stopStreaming();

    }

    class stream extends OpenCvPipeline {

        //Mat HSVFrame = new Mat();

        Mat outPut = new Mat();

        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        @Override
        public Mat processFrame(Mat input) {

            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double focalLength = 1120; // Approximate focal length in pixels for Logitech C920
            double realWidth = 8.9; // Real width of the sample in cm
            double realHeight = 3.8; //Real height of the sample in cm

            for (MatOfPoint contour : contours) {

                // Filter out small contours based on area
                if (Imgproc.contourArea(contour) > 1000) {  // Adjust this threshold based on real-world testing
                    // Use minAreaRect to get the minimum enclosing rectangle
                    RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                    // Get the four corners of the rectangle
                    Point[] rectPoints = new Point[4];
                    rotatedRect.points(rectPoints);

                    // Calculate distance to the object
                    double heightInPixels = rotatedRect.size.height; // Height of the rectangle in pixels
                    double distance = (focalLength * realWidth) / heightInPixels;
                    double distanceBasedOnHeight = (focalLength * realHeight) / heightInPixels;
                    double averageDistance = (distance + distanceBasedOnHeight) / 2;

                    // Calculate center point for distance text
                    Point center = new Point((rectPoints[0].x + rectPoints[2].x) / 2, (rectPoints[0].y + rectPoints[2].y) / 2);
                    String distanceText = String.format("%.2f cm", averageDistance);
                    Imgproc.putText(input, distanceText, center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);

                    // Draw the parallelogram on the input image
                    Imgproc.polylines(input, Arrays.asList(new MatOfPoint(rectPoints)), true, new Scalar(0, 255, 0), 2);
                }
            }

            input.copyTo(outPut);

            yellowMask.release();
            hierarchy.release();


            return outPut;

        }

    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Scalar lowerRed = new Scalar(110, 100, 100);
        Scalar upperRed = new Scalar(180, 255, 255);

        Mat redMask = new Mat();
        Core.inRange(hsvFrame, lowerRed, upperRed, redMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

        return redMask;
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);

        //controlHubCam.setPipeline(new ObjectTrackingCustom.YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        controlHubCam.setPipeline(new stream());
    }

}
