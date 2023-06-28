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
        RoadRunnerBotEntity testBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(90.00, 90.00, 90.00, 90.00, trackWidth)
                .setDimensions(robotWidth, robotHeight)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36.55, -65.79, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(36.09, -13.19), Math.toRadians(89.13))
                                .lineToConstantHeading(new Vector2d(15.94, -11.54))
                                .splineTo(new Vector2d(8.98, 15.02), Math.toRadians(104.69))
                                .splineTo(new Vector2d(-14.29, 14.84), Math.toRadians(180.45))
                                .splineTo(new Vector2d(-16.31, -11.54), Math.toRadians(90.00))
                                .splineTo(new Vector2d(-37.01, -14.66), Math.toRadians(188.56))
                                .splineTo(new Vector2d(-39.76, -32.43), Math.toRadians(270.00))
                                .splineTo(new Vector2d(-61.74, -65.22), Math.toRadians(236.16))
                        .build()
                );
        // to speed up ,SampleMecanumDrive.getVelocityConstraint(90, 90, 13.24),SampleMecanumDrive.getAccelerationConstraint(90)
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setAxesInterval(20)
                .setBackgroundAlpha(0.95f)
                .addEntity(testBot)
                //.addEntity(mySecondBot)
                .start();
    }
}