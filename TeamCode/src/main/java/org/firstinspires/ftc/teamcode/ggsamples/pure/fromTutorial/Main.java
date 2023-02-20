package org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.com.company.ComputerDebugging;
import org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.com.company.FloatPoint;
import org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.com.company.Robot;


public class Main {


    public static void main(String[] args) {
        new Main().run();
    }

    /**
     * The program runs here
     */
    public void run() {
        //this is a test of the coding
        ComputerDebugging computerDebugging = new ComputerDebugging();
        Robot robot = new Robot();
        OpMode opMode = new MyOpMode();
        opMode.init();

        ComputerDebugging.clearLogPoints();


        long startTime = System.currentTimeMillis();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        while (true) {

            opMode.loop();

            try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.update();
            ComputerDebugging.sendRobotLocation(robot);
            ComputerDebugging.sendLogPoint(new FloatPoint(Robot.worldXPosition, Robot.worldYPosition));
            ComputerDebugging.markEndOfUpdate();
        }
    }


}
