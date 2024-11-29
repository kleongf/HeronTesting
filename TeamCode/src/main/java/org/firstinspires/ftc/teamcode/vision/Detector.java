package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.util.SortOrder;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.RotatedRect;

import java.util.List;

public class Detector {
    private ColorBlobLocatorProcessor colorLocator;
    private VisionPortal portal;
    private List<ColorBlobLocatorProcessor.Blob> blobs;
    private int width = 320;
    private int height = 240;


    public Detector(HardwareMap hardwareMap) {
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(width, height))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    // placeholders until i use homography
    public double convertX(double x) {
        return 0.0;
    }

    public double convertY(double y) {
        return 0.0;
    }


    // this just tells how much to move based on where the robot is
    public double[] getClosestBlock() {
        blobs = colorLocator.getBlobs();
        if (!blobs.isEmpty()) {
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);
            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
            RotatedRect largestBoxFit = blobs.get(0).getBoxFit();
            double[] out = {convertX(largestBoxFit.center.x), convertY(largestBoxFit.center.y)};
            return out;
        } else {
            double[] out = {100.0, 100.0}; // choosing an impossible
            return out;
        }
    }

}