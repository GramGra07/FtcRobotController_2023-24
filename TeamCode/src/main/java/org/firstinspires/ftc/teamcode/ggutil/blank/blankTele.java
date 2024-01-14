package org.firstinspires.ftc.teamcode.ggutil.blank;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

@TeleOp
@Disabled
public class blankTele extends LinearOpMode {
    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, false);
        waitForStart();
        while (opModeIsActive()) {//while the op mode is active
            robot.doBulk();

        }
    }
}