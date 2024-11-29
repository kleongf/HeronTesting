package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.vision.Detector;

public class BlockSelector {
    private Detector detector;

    public BlockSelector(HardwareMap hardwareMap) {
        this.detector = new Detector(hardwareMap);
    }

    public double[] selectBlock() {
        List<ColorBlobLocatorProcessor.Blob> blobs = detector.colorLocator.getBlobs();

        // Split blobs by color
        List<ColorBlobLocatorProcessor.Blob> yellowBlobs = blobs.stream()
                .filter(blob -> blob.isColor(ColorRange.YELLOW))
                .collect(Collectors.toList());
        List<ColorBlobLocatorProcessor.Blob> blueBlobs = blobs.stream()
                .filter(blob -> blob.isColor(ColorRange.BLUE))
                .collect(Collectors.toList());

        // Prioritize yellow blocks if conditions are met
        if (!yellowBlobs.isEmpty()) {
            double yellowDistance = calculateDistance(yellowBlobs.get(0));
            double blueDistance = blueBlobs.isEmpty() ? Double.MAX_VALUE : calculateDistance(blueBlobs.get(0));

            if (yellowBlobs.size() >= 2 && yellowDistance <= 1.5 * blueDistance) {
                return detector.convertBlobToFieldCoords(yellowBlobs.get(0));
            }
        }

        // Default to blue blocks
        if (!blueBlobs.isEmpty()) {
            return detector.convertBlobToFieldCoords(blueBlobs.get(0));
        }

        // If no blocks are detected, return default position
        return new double[]{0.0, 0.0};
    }

    private double calculateDistance(ColorBlobLocatorProcessor.Blob blob) {
        RotatedRect boxFit = blob.getBoxFit();
        double fieldX = detector.convertX(boxFit.center.x);
        double fieldY = detector.convertY(boxFit.center.y);
        return Math.sqrt(fieldX * fieldX + fieldY * fieldY);
    }
}
