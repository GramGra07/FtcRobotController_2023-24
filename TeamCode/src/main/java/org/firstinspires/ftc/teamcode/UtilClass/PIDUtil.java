package org.firstinspires.ftc.teamcode.UtilClass;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDUtil {
    public PIDController pidController;
    public void initPIDController(double p, double i, double d, DcMotor motor, int target){
        pidController = new PIDController(
                p, // Proportional gain
                i, // Integral gain
                d // Derivative gain
        );
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runPIDController(DcMotor motor){
        motor.setPower(pidController.calculate(motor.getCurrentPosition(), motor.getTargetPosition()));
    }
}

