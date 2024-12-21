package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.shared.ExtendPIDF;
import org.firstinspires.ftc.teamcode.shared.Intake;
import org.firstinspires.ftc.teamcode.shared.LiftPIDF;

@Autonomous(name="Autonomous test", group="Autonomous")
public class AutoTest extends LinearOpMode {
    /*
    here is the plan:
    first, the robot stays in its position
    next, it extends the arm and gets the block
    it turns right (left depending on where you are placed)
    then you turn back and repeat two more times with the arm extended a bit more

    At this time the robot will be facing the correct direction (perpendicular to the edge)
    So now we just extend the arm up and grab the game piece

    next we move to the edge of the ledge
    then we move to the submersible while turning 180 degrees
    from my pov:
    move left end up (for now we will say by the same amount)
    then extend arm up to the clipping place
    outtake the block
    pull the arm back down
    move in the opposite vector and spin 180 degrees
    repeat

     */

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-36, 24, Math.toRadians(0));
        Pose2d submersiblePose = new Pose2d(-12, 0, Math.toRadians(0));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        DcMotorEx extendMotor = hardwareMap.get(DcMotorEx.class, "extendMotor");
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        ExtendPIDF extend = new ExtendPIDF(extendMotor);
        LiftPIDF lift = new LiftPIDF(liftMotor);
        Intake intake = new Intake(intakeMotor);


        TrajectoryActionBuilder turnRight = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(-90));

        TrajectoryActionBuilder turnLeft = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(90));

        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(24, 24), Math.toRadians(180));

        TrajectoryActionBuilder fromSubmersible = drive.actionBuilder(submersiblePose)
                .splineTo(new Vector2d(-24, -24), Math.toRadians(-180));

        if (isStopRequested()) return;

        // we need to add the wait one second thing here but whatever
        // this is for one cycle: the rest are copy and paste and extend.ExtendSecond()
        Actions.runBlocking(
                new SequentialAction(
                        extend.ExtendFirst(),
                        intake.IntakeForward(),
                        turnRight.build(),
                        intake.IntakeReverse(),
                        turnLeft.build()
                )
        );

        // need to turn and unextend the arm and lift it up
        Actions.runBlocking(
                new SequentialAction(
                       turnRight.build(),
                       extend.Retract(),
                        lift.LiftUp()
                )
        );


        Actions.runBlocking(
                new SequentialAction(
                    lift.LiftFirst(),
                        intake.IntakeForward(),
                        lift.LiftSecond(),
                        toSubmersible.build(),
                        lift.LiftFirstDown(),
                        intake.IntakeReverse(),
                        fromSubmersible.build()
                )
        );
    };
}
