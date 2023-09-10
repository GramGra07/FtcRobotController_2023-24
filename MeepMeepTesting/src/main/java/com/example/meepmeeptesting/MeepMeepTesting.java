package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(90));
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
                                .build()
                );

        // to speed up ,SampleMecanumDrive.getVelocityConstraint(90, 90, 13.24),SampleMecanumDrive.getAccelerationConstraint(90)
        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setAxesInterval(20)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                //.addEntity(mySecondBot)
                .start();
    }
}