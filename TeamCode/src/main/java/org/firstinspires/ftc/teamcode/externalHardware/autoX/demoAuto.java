package org.firstinspires.ftc.teamcode.externalHardware.autoX;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.externalHardware.autoHardware;

@Autonomous(name = "demoAuto", group = "Robot")
//@Disabled
public class demoAuto extends LinearOpMode {
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAuto(hardwareMap);
        if (opModeIsActive()) {//while the op mode is active
            robot.encoderDrive(1, 10, 10, 5);
        }
    }
}