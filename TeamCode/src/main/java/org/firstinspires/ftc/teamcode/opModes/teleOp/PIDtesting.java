package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class PIDtesting extends LinearOpMode {
    public static double p = 0.5;
    public static double i = 0.0;
    public static double d = 0.0;

    @Override
    public void runOpMode() {
        // Initialize the PID controller

        PIDController pidController = new PIDController(
                p, // Proportional gain
                i, // Integral gain
                d // Derivative gain
        );

        // Initialize the motor
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setTargetPosition(10000);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime elapsedTime = new ElapsedTime();
        waitForStart();
        while (opModeIsActive() && motor.getCurrentPosition() < motor.getTargetPosition()) {
            telemetry.addData("val", motor.getCurrentPosition());
            telemetry.update();
            // Calculate the output of the PID controller
            double output = pidController.calculate(motor.getCurrentPosition(), motor.getTargetPosition());
            // Set the power of the motor based on the output of the PID controller
            motor.setPower(output);
            // Update the elapsed time
            elapsedTime.reset();
        }
    }
}
