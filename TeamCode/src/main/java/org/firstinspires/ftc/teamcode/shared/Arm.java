package org.firstinspires.ftc.teamcode.shared;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Arm {
    private DcMotorEx lift;
    private DcMotorEx extend;
    private AnalogInput angle;
    //AnalogInput input = new AnalogInput(0);

    public Arm(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        extend = hardwareMap.get(DcMotorEx.class, "extendMotor");
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setDirection(DcMotorSimple.Direction.FORWARD);
        angle = hardwareMap.get(AnalogInput.class, "angle");
    }

    public double thetaDegrees(double input) {
        return (input % 3.3) * (360 / 3.3);
    }

    // we need one for lifting the arm up to the first basket and the second basket.
    // if we need more, we can copy and paste

    public class LiftLevelOne implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPower(0.8);
                initialized = true;
            }
            // change to angle input
            double theta = thetaDegrees(angle.getVoltage());
            packet.put("theta", theta);
            if (theta < 60.0) {
                return true;
            } else {
                lift.setPower(0);
                return false;
            }
        }
    }

    public Action liftLevelOne() {
        return new LiftLevelOne();
    }


    public class LiftLevelTwo implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPower(0.8);
                initialized = true;
            }
            // change to angle input
            double theta = thetaDegrees(angle.getVoltage());
            packet.put("theta", theta);
            if (theta < 80.0) {
                return true;
            } else {
                lift.setPower(0);
                return false;
            }
        }
    }

    public Action liftLevelTwo() {
        return new LiftLevelTwo();
    }


    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPower(-0.8);
                initialized = true;
            }

            double theta = thetaDegrees(angle.getVoltage());
            packet.put("theta", theta);
            if (theta > 5.0) {
                return true;
            } else {
                lift.setPower(0);
                return false;
            }
        }
    }
    public Action liftDown() {
        return new LiftDown();
    }

    public class ExtendLevelTwo implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                extend.setPower(0.8);
                initialized = true;
            }

            double pos = extend.getCurrentPosition();
            packet.put("extend", pos);
            if (pos < 2000.0) {
                return true;
            } else {
                extend.setPower(0);
                return false;
            }
        }
    }

    public Action ExtendLevelTwo() {
        return new ExtendLevelTwo();
    }

    public class ExtendLevelOne implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                extend.setPower(0.8);
                initialized = true;
            }

            double pos = extend.getCurrentPosition();
            packet.put("extend", pos);
            if (pos < 3000.0) {
                return true;
            } else {
                extend.setPower(0);
                return false;
            }
        }
    }

    public Action ExtendLevelOne() {
        return new ExtendLevelOne();
    }

    public class Retract implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                extend.setPower(-0.8);
                initialized = true;
            }

            double pos = extend.getCurrentPosition();
            if (pos > 100.0) {
                return true;
            } else {
                extend.setPower(0);
                return false;
            }
        }
    }
    public Action Retract() {
        return new Retract();
    }
}
