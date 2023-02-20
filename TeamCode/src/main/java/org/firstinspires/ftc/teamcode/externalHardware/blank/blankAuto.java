package org.firstinspires.ftc.teamcode.externalHardware.blank;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.externalHardware.autoHardware;

@Autonomous(name = "blankAuto", group = "Robot")
@Disabled
public class blankAuto extends LinearOpMode {
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAuto(hardwareMap);
        if (opModeIsActive()) {//while the op mode is active

        }
    }
}