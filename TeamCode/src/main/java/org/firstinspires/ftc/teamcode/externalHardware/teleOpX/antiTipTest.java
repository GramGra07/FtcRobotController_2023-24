package org.firstinspires.ftc.teamcode.externalHardware.teleOpX;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.externalHardware.HardwareConfig;
import org.firstinspires.ftc.teamcode.externalHardware.MathFunctions;

@TeleOp(name = "antiTipTest", group = "Robot")
//@Disabled
public class antiTipTest extends LinearOpMode {
    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("heading", robot.angles.firstAngle);
        telemetry.addData("roll", robot.angles.secondAngle);
        telemetry.addData("pitch", robot.angles.thirdAngle);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {//while the op mode is active
            MathFunctions.antiTip();
            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.addData("roll", robot.angles.secondAngle);
            telemetry.addData("pitch", robot.angles.thirdAngle -180);
            telemetry.update();
        }
    }
}