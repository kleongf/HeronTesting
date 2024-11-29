package org.firstinspires.ftc.teamcode.shared;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoMotor {
    private Servo servoMotor;
    public ServoMotor(Servo motor) {
        servoMotor = motor;
    }
    public class Turn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            servoMotor.setPosition(0.5);
            return false;
        }
    }

    public Action Turn() {
        return new Turn();
    }
    public class Straighten implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            servoMotor.setPosition(0);
            return false;
        }
    }
    public Action Straighten() {
        return new Straighten();
    }
}
