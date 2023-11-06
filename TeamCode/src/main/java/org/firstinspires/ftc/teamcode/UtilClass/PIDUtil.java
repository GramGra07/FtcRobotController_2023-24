package org.firstinspires.ftc.teamcode.UtilClass;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDUtil {

    public static void initPIDController(double p, double i, double d, DcMotor motor, int target) {
        new PIDController(
                p, // Proportional gain
                i, // Integral gain
                d // Derivative gain
        );
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void setupPIDMotor(DcMotor motor, int target){
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void calculatePID(DcMotor motor,PIDController pidController) {
        motor.setPower(pidController.calculate(motor.getCurrentPosition(), motor.getTargetPosition()));
    }

}

