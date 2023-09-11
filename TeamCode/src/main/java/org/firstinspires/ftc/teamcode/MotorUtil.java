package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

public class MotorUtil {
    public void resetEncoder(List<DcMotor> motors){
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void runWithoutEncoder(List<DcMotor> motor){
        for (DcMotor motors : motor) {
            motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void runWithEncoder(List<DcMotor> motor){
        for (DcMotor motors : motor) {
            motors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void setDirectionR(List<DcMotor> motor){
        for (DcMotor motors : motor) {
            motors.setDirection(DcMotor.Direction.REVERSE);
        }
    }
    public void setDirectionF(List<DcMotor> motor){
        for (DcMotor motors : motor) {
            motors.setDirection(DcMotor.Direction.FORWARD);
        }
    }
    public void zeroPowerBrake(List<DcMotor> motor){
        for (DcMotor motors : motor) {
            motors.setZeroPowerBehavior(BRAKE);
        }
    }
    public double getCurrentPose(DcMotor motor){
        return motor.getCurrentPosition();
    }
    public void setTargetPose(DcMotor motor, int pose){
        motor.setTargetPosition(pose);
    }
    public void runToPose(DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
