package org.firstinspires.ftc.teamcode.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;

@Autonomous
@Disabled
public class tester extends LinearOpMode {
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initAuto(hardwareMap);

        if(isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()){
            Vision.telemetryAprilTag(this);
            Vision.telemetryTfod(this);
            telemetry.update();
        }
    }
}