package org.firstinspires.ftc.teamcode.ggsamples.testOpModes;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ggsamples.nonx.teleOp.robotCentric;


import java.util.Objects;

@TeleOp(name = "distanceSensorCalibrator", group = "robot")
@Disabled
public class distanceSensorCalibrator extends robotCentric {
    //distance
    //range
    private DistanceSensor rDistance;
    public static double rOffset = 2.2;
    private DistanceSensor lDistance;
    public static double lOffset = 0.7;//-5.1;
    private DistanceSensor fDistance;
    public static double fOffset = 6;
    private DistanceSensor bDistance;
    public static double bOffset = -2.9;

    @Override
    public void runOpMode() {
        rDistance = hardwareMap.get(DistanceSensor.class, "rDistance");
        lDistance = hardwareMap.get(DistanceSensor.class, "lDistance");
        fDistance = hardwareMap.get(DistanceSensor.class, "fDistance");
        bDistance = hardwareMap.get(DistanceSensor.class, "bDistance");
        DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");//getting the motorFrontLeft motor
        DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");//getting the motorBackLeft motor
        DcMotor motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");//getting the motorFrontRight motor
        DcMotor motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");//getting the motorBackRight motor
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);//setting the motorFrontRight direction
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);//setting the motorBackRight direction

        motorBackRight.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);
        motorFrontLeft.setZeroPowerBehavior(BRAKE);
        calibrateDist("f", 6, 6, 6, 6);
        calibrateDist("l", 6, 6, 6, 6);
        waitForStart();
        while (opModeIsActive()) {
            //distance
            telemetry.addData("lOffset", lOffset);
            telemetry.addData("rOffset", rOffset);
            telemetry.addData("fOffset", fOffset);
            telemetry.addData("bOffset", bOffset);
            telemetry.addData("f", fDistance.getDistance(DistanceUnit.CM) + fOffset);
            telemetry.addData("b", bDistance.getDistance(DistanceUnit.CM) + bOffset);
            telemetry.addData("l", lDistance.getDistance(DistanceUnit.CM) + lOffset);
            telemetry.addData("r", rDistance.getDistance(DistanceUnit.CM) + rOffset);
            telemetry.update();

        }
    }

    public void calibrateDist(String cal, double knownDistl, double knownDistr, double knownDistf, double knownDistb) {
        if (Objects.equals(cal, "l")) {
            lOffset = knownDistl - lDistance.getDistance(DistanceUnit.CM);
        } else if (Objects.equals(cal, "r")) {
            rOffset = knownDistr - rDistance.getDistance(DistanceUnit.CM);
        } else if (Objects.equals(cal, "f")) {
            fOffset = knownDistf - fDistance.getDistance(DistanceUnit.CM);
        } else if (Objects.equals(cal, "b")) {
            bOffset = knownDistb - bDistance.getDistance(DistanceUnit.CM);
        }

    }
}