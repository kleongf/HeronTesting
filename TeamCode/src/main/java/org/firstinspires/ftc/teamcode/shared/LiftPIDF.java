package org.firstinspires.ftc.teamcode.shared;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class LiftPIDF {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double ticksInDegree = 757 / 180.0;
    private DcMotorEx liftMotor;

    public LiftPIDF(DcMotorEx motor) {
        controller = new PIDController(p, i, d);
        liftMotor = motor;
    }

    public void setTarget(int t) {
        target = t;
    }

    public void loop() {
        controller.setPID(p, i, d);
        int armPos = liftMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.sin(Math.toRadians(target / ticksInDegree)) * f;

        double power = pid + ff;

        liftMotor.setPower(power);
    }
}
