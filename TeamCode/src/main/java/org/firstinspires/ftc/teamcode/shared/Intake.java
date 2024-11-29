package org.firstinspires.ftc.teamcode.shared;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx intakeMotor;
    public Intake(DcMotorEx motor) {
        intakeMotor = motor;
    }
    public class IntakeForward implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            intakeMotor.setPower(0.8);
            return false;
        }
    }

    public Action IntakeForward() {
        return new IntakeForward();
    }
    public class IntakeReverse implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            intakeMotor.setPower(-0.8);
            return false;
        }
    }

    public Action IntakeReverse() {
        return new IntakeReverse();
    }

    public class IntakeStop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            intakeMotor.setPower(0);
            return false;
        }
    }

    public Action IntakeStop() {
        return new IntakeStop();
    }
}
