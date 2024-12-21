package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.shared.ExtendPIDF;
import org.firstinspires.ftc.teamcode.shared.Intake;
import org.firstinspires.ftc.teamcode.shared.LiftPIDF;
import org.firstinspires.ftc.teamcode.shared.ServoMotor;
import org.firstinspires.ftc.teamcode.vision.Detector;

// this could all be wrong we probably want a normal opmode since its iterative
// also in the loop() function i need to power the motors :skull:

@Autonomous(name = "Autonomous 1", group = "Autonomous")
public class Auto extends LinearOpMode {

    // this is simple: we will use SequentialAction, run the trajectory, etc.
    // something important to know about actions:
    // public boolean run(TelemetryPacket packet): Code to run repeatedly while the method returns true
    // so we return false from the method when we are done
    // we should also use the actions' builtin wait function

    // wondering if it is a good idea to keep power to lift and extend all the time
    // sequential action will only call setPosition() and maybe we wait
    /*
        *** intake will be off/reversed at first ***
        1. drive to closest block
            a. we will have a global variable from the vision that constantly outputs closest block
            b. this trajectory will drive to the closest block (duh)
        2. intake the block
            a. intake turns on (run this forever idk)
        3. drive to goal post
        4. lift arm up (dont wait for this to finish running because it is pid)
        5. extend arm
        6. rotate servo to dispense block
        (wait a bit of time)


        7. straighten servo again
        8. retract arm
        9. lift arm down
        10. drive back to submersible
     */
    public enum CameraState {
        CAMERA_ON,
        CAMERA_OFF
    }

    @Override
    public void runOpMode() {
        Detector detector = new Detector(hardwareMap);
        double[] selectedBlock = detector.getClosestBlock();

        Pose2d initialPose = new Pose2d(-30, 10, Math.toRadians(0));
        Pose2d blockPose = new Pose2d(selectedBlock[0], selectedBlock[1], Math.toRadians(0));
        Pose2d bucketPose = new Pose2d(-68, 68, Math.toRadians(90));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        DcMotorEx extendMotor = hardwareMap.get(DcMotorEx.class, "extendMotor");
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        Servo servoMotor = hardwareMap.get(Servo.class, "servoMotor");

        ExtendPIDF extend = new ExtendPIDF(extendMotor);
        LiftPIDF lift = new LiftPIDF(liftMotor);
        ServoMotor servo = new ServoMotor(servoMotor);
        Intake intake = new Intake(intakeMotor);



        // we need to change this but the logic is there
        // i dont know roadrunner well so i made it simple
        TrajectoryActionBuilder initialToBlock = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToY(selectedBlock[1])
                .setTangent(Math.toRadians(0))
                .lineToX(selectedBlock[0])
                .waitSeconds(1);

        TrajectoryActionBuilder blockToBucket = drive.actionBuilder(blockPose)
                .setTangent(Math.toRadians(90))
                .lineToY(68)
                .setTangent(Math.toRadians(0))
                .lineToX(-68)
                .waitSeconds(1);
        TrajectoryActionBuilder bucketToInitial = drive.actionBuilder(bucketPose)
                .setTangent(Math.toRadians(90))
                .lineToY(-30)
                .setTangent(Math.toRadians(0))
                .lineToX(10)
                .waitSeconds(1);

        TrajectoryActionBuilder waitOneSecond = drive.actionBuilder(bucketPose)
                .waitSeconds(1);
        // make one trajectory here that moves robot to the block
        // another moves the robot to the bucket
        // another moves the robot back
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        initialToBlock.build(),
                        intake.IntakeForward(),
                        blockToBucket.build(),
                        lift.LiftUp(),
                        extend.ExtendFirst(),
                        servo.Turn(),
                        waitOneSecond.build(),
                        servo.Straighten(),
                        extend.Retract(),
                        lift.LiftDown(),
                        bucketToInitial.build()
                )
        );
    }
}
