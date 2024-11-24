package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shared.ExtendPIDF;
import org.firstinspires.ftc.teamcode.shared.LiftPIDF;


@TeleOp(name="Main TeleOp", group="Linear OpMode")
public class mainTeleop extends OpMode {

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_RETRACT,
        LIFT_UP,
    };

    public enum IntakeState {
        INTAKE_ON,
        INTAKE_OFF,
        INTAKE_TILTED,
        INTAKE_STRAIGHT
    };

    public enum ClipState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_RETRACT,
        LIFT_UP,
    }

    // we need another state for clipping (done)
    // it will be similar but not the same

    // the servo has setPosition from 0 to 1 (0 to 180 degrees)
    // therefore 90 degrees is setPosition(0.5)
    ElapsedTime intakeTimer = new ElapsedTime();

    LiftState liftState = LiftState.LIFT_START;
    IntakeState intakeState = IntakeState.INTAKE_ON;
    ClipState clipState = ClipState.LIFT_START;

    LiftPIDF liftPIDF;
    ExtendPIDF extendPIDF;


    // change these based on constants, these are simply placeholders for now
    // these will be target positions
    public int LIFT_HIGH = 1000;
    public int LIFT_LOW = 0;
    public int EXTEND_HIGH = 1000;
    public int EXTEND_LOW = 0;
    public int LIFT_CLIP_HIGH = 700;
    public int LIFT_CLIP_LOW = 0;
    public int EXTEND_CLIP_HIGH = 700;
    public int EXTEND_CLIP_LOW = 0;

    public double SERVO_TILTED = 0.5;
    public double SERVO_STRAIGHT = 0;
    public double INTAKE_TIME = 2;


    public DcMotorEx liftMotor;
    public DcMotorEx extendMotor;
    public DcMotorEx intakeMotor;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private Servo servoMotor;

    @Override
    public void init() {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        extendMotor = hardwareMap.get(DcMotorEx.class, "extendMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        servoMotor = hardwareMap.get(Servo.class, "servoMotor");

        liftPIDF = new LiftPIDF(liftMotor);
        extendPIDF = new ExtendPIDF(extendMotor);

        intakeTimer.reset();
    }

    @Override
    public void loop() {
        // when x is pressed, the angle is changed and the arm extends
        // when x is pressed again, the arm and angle are reset
        switch (liftState) {
            case LIFT_START:
                // Waiting for some input
                if (gamepad1.x) {
                    // x is pressed, start extending
                    // this is the one that changes angle
                    // will need pid but can implement later here
                    // change the setTargetPosition() calls to simply set the target
                    // at the very end we will apply the designate power to our motors
                    // for example here we would say liftTarget = LIFT_HIGH
                    liftPIDF.setTarget(LIFT_HIGH);
                    liftState = LiftState.LIFT_UP;
                }
                break;
            case LIFT_UP:
                // check if the lift has finished extending,
                // otherwise do nothing.
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_HIGH) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // extend motor doesn't really need pid
                    // however it would not be stupid
                    // we can change this a bit but this is what we need to do
                    extendPIDF.setTarget(EXTEND_HIGH);
                    liftState = LiftState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                if (gamepad1.x) {
                    if (Math.abs(extendMotor.getCurrentPosition() - EXTEND_LOW) < 10) {
                        liftPIDF.setTarget(LIFT_LOW);
                        liftState = LiftState.LIFT_RETRACT;
                    }
                    extendPIDF.setTarget(EXTEND_LOW);
                }
                break;
            case LIFT_RETRACT:
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_LOW) < 10) {
                    liftState = LiftState.LIFT_START;
                }
                break;

            default:
                // should never be reached, as liftState should never be null
                liftState = LiftState.LIFT_START;
        }

        // when a is pressed, the intake moves to the side, reverses direction, and moves back

        switch (intakeState) {
            // lets think about it
            // you want to tilt the servo
            // turn the intake off (reverse it)
            // untilt the servo
            // turn the intake on

            case INTAKE_ON:
                intakeMotor.setPower(0.8);
                if (gamepad1.a) {
                    servoMotor.setPosition(0.5);
                    intakeState = IntakeState.INTAKE_TILTED;
                }
                break;

            case INTAKE_TILTED:
                if (Math.abs(servoMotor.getPosition() - SERVO_TILTED) < 0.05) {
                    intakeMotor.setPower(-0.8);
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_OFF;
                }
                break;

            case INTAKE_OFF:
                if (intakeTimer.seconds() >= INTAKE_TIME) {
                    // robot waits until the piece is deposited
                    servoMotor.setPosition(0);
                    intakeState = IntakeState.INTAKE_STRAIGHT;
                }
                break;

            case INTAKE_STRAIGHT:
                if (Math.abs(servoMotor.getPosition() - SERVO_STRAIGHT) < 0.05) {
                    intakeMotor.setPower(0.8);
                    intakeState = IntakeState.INTAKE_ON;
                }
                break;

            default:
                intakeState = IntakeState.INTAKE_ON;
        }
        // when b is pressed, the robot extends arm and changes angle
        // when b is pressed again, it retracts back to original position
        switch (clipState) {
            case LIFT_START:
                // Waiting for some input
                if (gamepad1.b) {
                    // x is pressed, start extending
                    // this is the one that changes angle
                    // will need pid but can implement later here
                    // change the setTargetPosition() calls to simply set the target
                    // at the very end we will apply the designate power to our motors
                    // for example here we would say liftTarget = LIFT_HIGH
                    liftPIDF.setTarget(LIFT_CLIP_HIGH);
                    clipState = ClipState.LIFT_UP;
                }
                break;
            case LIFT_UP:
                // check if the lift has finished extending,
                // otherwise do nothing.
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_CLIP_HIGH) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // extend motor doesn't really need pid
                    // however it would not be stupid
                    // we can change this a bit but this is what we need to do
                    extendPIDF.setTarget(EXTEND_CLIP_HIGH);
                    clipState = ClipState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                if (gamepad1.b) {
                    if (Math.abs(extendMotor.getCurrentPosition() - EXTEND_CLIP_LOW) < 10) {
                        liftPIDF.setTarget(LIFT_CLIP_LOW);
                        clipState = ClipState.LIFT_RETRACT;
                    }
                    extendPIDF.setTarget(EXTEND_CLIP_LOW);
                }
                break;
            case LIFT_RETRACT:
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_CLIP_LOW) < 10) {
                    clipState = ClipState.LIFT_START;
                }
                break;

            default:
                // should never be reached, as liftState should never be null
                clipState = ClipState.LIFT_START;
        }

        // call motor set power here for PID
        liftPIDF.loop();
        extendPIDF.loop();
        // only thing that the finite state machine is doing is setting the target postiion

        double y = -this.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = this.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = this.gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }

}
