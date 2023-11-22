package org.firstinspires.ftc.teamcode.UtilClass;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDUtil {

    public static void calculatePID(DcMotor motor,PIDController pidController) {
        motor.setPower(pidController.calculate(motor.getCurrentPosition(), motor.getTargetPosition()));
    }

}

