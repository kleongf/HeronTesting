package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "Pinpoint Test", group = "Autonomous")
public class PinpointTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        int trajectoryChosen = 0;
        double deltaX = 0;
        double deltaY = 0;
        // change this to our starting pose
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        // change this to the pose at the bucket
        Pose2d scoringPose = new Pose2d(60, 60, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        // in theory this should drive the robot to the block and then go to the scoring place
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineTo(new Vector2d(deltaX, deltaY), Math.PI / 2)
                .waitSeconds(2)
                .setTangent(Math.toRadians(315))
                .lineToY(60)
                .setTangent(Math.toRadians(0))
                .lineToX(60);


        Action trajectoryAction = tab1.build();
        if (isStopRequested()) return;

        // add other things too
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction
                )
        );
    }
}

