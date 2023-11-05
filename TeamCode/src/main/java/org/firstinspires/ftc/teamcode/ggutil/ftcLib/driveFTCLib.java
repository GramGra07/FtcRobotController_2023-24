package org.firstinspires.ftc.teamcode.ggutil.ftcLib;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class driveFTCLib extends LinearOpMode {
    public Motor motor1 = null;
    public Motor motor2 = null;
    public Motor motor3 = null;
    public Motor motor4 = null;

    public void runOpMode() {
        // Initialize the gamepad
        new GamepadEx(gamepad1);//
        motor1 = new Motor(hardwareMap, "motorFrontLeft");
        motor2 = new Motor(hardwareMap, "motorFrontRight");
        motor3 = new Motor(hardwareMap, "motorBackLeft");
        motor4 = new Motor(hardwareMap, "motorBackRight");
        // Initialize the mecanum drive
        MecanumDrive mecanumDrive = new MecanumDrive(
                motor1, motor2, motor3, motor4
        );

// Set the mecanum drive power based on the gamepad input
        while (opModeIsActive()) {
            double yControl = -gamepad1.left_stick_y;
            double xControl = gamepad1.left_stick_x;
            double slow = 1;
            double turn = -gamepad1.right_stick_x;
            mecanumDrive.driveRobotCentric(xControl, yControl, turn);
        }
    }
}
