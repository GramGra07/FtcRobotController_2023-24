package org.firstinspires.ftc.teamcode.pure.fromTutorial;

import static org.firstinspires.ftc.teamcode.pure.fromTutorial.RobotMovement.followCurve;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous(name = "PurePursuit", group = "Robot")
@Disabled
public class MyOpMode extends LinearOpMode {
    private static Motor fL, fR, bL, bR;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = new Motor(hardwareMap, "motorFrontLeft");
        fR = new Motor(hardwareMap, "motorFrontRight");
        bL = new Motor(hardwareMap, "motorBackLeft");
        bR = new Motor(hardwareMap, "motorBackRight");
        if (opModeIsActive()) {
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            //358 is full length
            //30 per tile
            //follow distance = lookahead distance
            allPoints.add(new CurvePoint(0, 30, 1, 0.8, 50, 50, 0, 0));
            //add more of ^^^
            followCurve(allPoints, 90);
        }
    }

    public static void applyPower(double frontRightPower, double backRightPower, double frontLeftPower, double backLeftPower) {
        fR.set(frontRightPower);
        bR.set(backRightPower);
        fL.set(frontLeftPower);
        bL.set(backLeftPower);
    }
}
