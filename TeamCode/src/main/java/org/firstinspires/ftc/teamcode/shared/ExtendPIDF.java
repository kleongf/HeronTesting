package org.firstinspires.ftc.teamcode.shared;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ExtendPIDF {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    // ublic static double f = 0;
    public static int target = 0;
    // we dont care because there is no angle

    private DcMotorEx extendMotor;

    public ExtendPIDF(DcMotorEx motor) {
        controller = new PIDController(p, i, d);
        extendMotor = motor;
    }

    public void setTarget(int t) {
        target = t;
    }

    public void loop() {
        controller.setPID(p, i, d);
        int armPos = extendMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        // we dont care about angle here either or feed forward, its already taken care of
        // double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;
        double power = pid;

        extendMotor.setPower(power);
    }

    public class ExtendFirst implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            setTarget(1000);
            return false;
        }
    }
    public Action ExtendFirst() {
        return new ExtendFirst();
    }

    public class ExtendSecond implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            setTarget(2000);
            return false;
        }
    }
    public Action ExtendSecond() {
        return new ExtendSecond();
    }

    public class ExtendThird implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            setTarget(3000);
            return false;
        }
    }
    public Action ExtendThird() {
        return new ExtendThird();
    }

    public class Retract implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            setTarget(0);
            return false;
        }
    }
    public Action Retract() {
        return new Retract();
    }
}