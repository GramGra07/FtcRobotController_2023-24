package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(90));
        int maxVel = 57;
        int maxAccel = 60;
        int maxAngVel = 180;
        int maxAngAccel = 360;
        double trackWidth = 13.24;
        double robotWidth = 16;
        double robotHeight = 18;
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(maxVel, maxAccel, Math.toRadians(maxAngVel), Math.toRadians(maxAngAccel), trackWidth)
                .setDimensions(robotWidth, robotHeight)//bot width and height
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(60,60), Math.toRadians(90))
                                .build()
                );
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(maxVel, maxAccel, Math.toRadians(maxAngVel), Math.toRadians(maxAngAccel), trackWidth)
                .setDimensions(robotWidth, robotHeight)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .forward(30)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                //.addEntity(mySecondBot)
                .start();
    }
}