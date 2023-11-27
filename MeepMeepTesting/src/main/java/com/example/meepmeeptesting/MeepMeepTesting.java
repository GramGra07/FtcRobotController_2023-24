package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(38,10,  Math.toRadians(90));
        int maxVel = 57;
        int maxAccel = 60;
        int maxAngVel = 360;
        int maxAngAccel = 360;
        double trackWidth = 13.24;
        double robotWidth = 16;
        double robotHeight = 18;
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(maxVel, maxAccel, 60, 60, trackWidth)
                .setDimensions(robotWidth, robotHeight)//bot width and height
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(12, -60, Math.toRadians(-90)))
                                .addDisplacementMarker(() -> {
                                })
                                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(35, 48, Math.toRadians(90)))
                                .build()
                );

        // to speed up ,SampleMecanumDrive.getVelocityConstraint(90, 90, 13.24),SampleMecanumDrive.getAccelerationConstraint(90)
        Image img = null;
        try {
            img = ImageIO.read(new File("/Users/gradengentry/Desktop/robotics/field.png"));
        }
        // graden: "/Users/gradengentry/Desktop/robotics/field.png"
        // chase: "C:\Users\bubba\OneDrive\Desktop\Robotics\field.png"
        catch (IOException e) {
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setAxesInterval(20)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                //.addEntity(mySecondBot)
                .start();
    }
}