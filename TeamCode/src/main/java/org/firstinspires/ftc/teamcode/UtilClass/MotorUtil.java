package org.firstinspires.ftc.teamcode.UtilClass;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

public class MotorUtil {
    public static void runWithoutEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void runWithEncoder(List<DcMotor> motor) {
        for (DcMotor motors : motor) {
            motors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public static void setDirectionR(DcMotor motor) {
        motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public static void setDirectionF(List<DcMotor> motor) {
        for (DcMotor motors : motor) {
            motors.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public static void zeroPowerBrake(DcMotor motor) {
        motor.setZeroPowerBehavior(BRAKE);
    }

    public static double getCurrentPose(DcMotor motor) {
        return motor.getCurrentPosition();
    }

    public static void setTargetPose(DcMotor motor, int pose) {
        motor.setTargetPosition(pose);
    }

    public static void runToPose(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
