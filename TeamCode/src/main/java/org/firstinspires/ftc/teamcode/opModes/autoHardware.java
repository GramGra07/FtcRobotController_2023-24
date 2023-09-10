package org.firstinspires.ftc.teamcode.opModes;

import static android.os.SystemClock.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Blink;

public class autoHardware extends HardwareConfig {//auto version of hardware config

    public static Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));
    HardwareMap hardwareMap = null;

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public autoHardware(LinearOpMode opMode) {
        super(opMode);
        myOpMode = opMode;
    }

    public void initAuto(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        init(ahwMap);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        myOpMode.waitForStart();
        timer.reset();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(Blink.getColor()));
    }


}
