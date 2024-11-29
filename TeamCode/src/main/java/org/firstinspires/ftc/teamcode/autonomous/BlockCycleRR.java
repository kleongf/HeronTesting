package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.Detector;
import org.firstinspires.ftc.teamcode.vision.BlockSelector;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlockCycleRR {
    private PinpointDrive drive;
    private Detector detector;
    private BlockSelector selector;
    private Telemetry telemetry;

    // Forbidden zone boundaries
    private static final double FORBIDDEN_X_MIN = -24;
    private static final double FORBIDDEN_X_MAX = 24;
    private static final double FORBIDDEN_Y_MIN = -40;
    private static final double FORBIDDEN_Y_MAX = 40;

    // Constructor
    public BlockCycleRR(PinpointDrive drive, Detector detector, Telemetry telemetry) {
        this.drive = drive;
        this.detector = detector;
        this.selector = new BlockSelector(detector);
        this.telemetry = telemetry;
    }

    // Method to avoid forbidden zones
    private Trajectory generateSafeTrajectory(Pose2d startPose, Pose2d targetPose) {
        Trajectory trajectory = drive.trajectoryBuilder(startPose).lineToLinearHeading(targetPose).build();

        while (true) {
            boolean collisionDetected = false;

            // Check each segment for forbidden zone collisions
            for (Pose2d point : trajectory.getPath().getPoints()) {
                if (isInForbiddenZone(point)) {
                    collisionDetected = true;

                    // Add intermediate waypoint just outside forbidden zone
                    Pose2d adjustedPoint = adjustPoint(point);
                    trajectory = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(adjustedPoint)
                            .lineToLinearHeading(targetPose)
                            .build();
                    break; // Re-check trajectory
                }
            }

            if (!collisionDetected) {
                break; // No collisions detected
            }
        }

        return trajectory;
    }

    // Check if a point is in the forbidden zone
    private boolean isInForbiddenZone(Pose2d point) {
        double x = point.getX();
        double y = point.getY();
        return x >= FORBIDDEN_X_MIN && x <= FORBIDDEN_X_MAX &&
                y >= FORBIDDEN_Y_MIN && y <= FORBIDDEN_Y_MAX;
    }

    // Adjust a point to just outside the forbidden zone
    private Pose2d adjustPoint(Pose2d collisionPoint) {
        double x = collisionPoint.getX();
        double y = collisionPoint.getY();

        if (x >= FORBIDDEN_X_MIN && x <= FORBIDDEN_X_MAX) {
            x = (x > 0) ? FORBIDDEN_X_MAX + 4 : FORBIDDEN_X_MIN - 4;
        }
        if (y >= FORBIDDEN_Y_MIN && y <= FORBIDDEN_Y_MAX) {
            y = (y > 0) ? FORBIDDEN_Y_MAX + 4 : FORBIDDEN_Y_MIN - 4;
        }

        return new Pose2d(x, y, collisionPoint.getHeading());
    }

    public void startCycle() {
        while (true) {
            // Step 1: Drive to the side of the sub area
            Pose2d sidePose = new Pose2d(-30, 10, Math.toRadians(0));
            Trajectory toSide = generateSafeTrajectory(drive.getPoseEstimate(), sidePose);
            drive.followTrajectory(toSide);

            // Step 2: Select the closest block using heuristic
            double[] selectedBlock = selector.selectBlock();
            Pose2d blockPose = new Pose2d(selectedBlock[0], selectedBlock[1], Math.toRadians(0));

            // Step 3: Drive to the nearest allowed point to the selected block
            double xRestricted = Math.max(-24, Math.min(blockPose.getX(), -24));
            double yRestricted = Math.max(-16, Math.min(blockPose.getY(), 16));
            Pose2d restrictedPose = new Pose2d(xRestricted, yRestricted, blockPose.getHeading());

            Trajectory toBlock = generateSafeTrajectory(drive.getPoseEstimate(), restrictedPose);
            drive.followTrajectory(toBlock);

            // Simulate arm grabbing the block
            telemetry.addData("Status", "Grabbing block at %s", restrictedPose);
            telemetry.update();
            try {
                Thread.sleep(2000); // Placeholder for grabbing block
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            // Step 4: Drive to the bucket and orient toward it
            Pose2d bucketPose = new Pose2d(-68, 68, Math.toRadians(90));
            Trajectory toBucket = generateSafeTrajectory(drive.getPoseEstimate(), bucketPose);
            drive.followTrajectory(toBucket);

            telemetry.addData("Status", "Dropping block at bucket");
            telemetry.update();
            try {
                Thread.sleep(2000); // Placeholder for dropping block
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public static void main(String[] args) {
        SampleMecanumDrive drive = new SampleMecanumDrive(null);
        Detector detector = new Detector(null); // Pass hardwareMap here in real usage
        Telemetry telemetry = null; // Initialize with actual telemetry instance

        BlockCycleRR blockCycle = new BlockCycleRR(drive, detector, telemetry);
        blockCycle.startCycle();
    }
}
