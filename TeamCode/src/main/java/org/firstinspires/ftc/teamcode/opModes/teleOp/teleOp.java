package org.firstinspires.ftc.teamcode.opModes.teleOp;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.aprilTagProcessor;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.visionPortal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;


@TeleOp(group = "a")
//@Disabled//disabling the opmode
public class teleOp extends LinearOpMode {//declaring the class
    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void runOpMode() {//if opmode is started
        robot.init(hardwareMap, false);
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        waitForStart();
        HardwareConfig.timer.reset();
        while (opModeIsActive()) {//while the op mode is active
            robot.doBulk();
        }
    }
}