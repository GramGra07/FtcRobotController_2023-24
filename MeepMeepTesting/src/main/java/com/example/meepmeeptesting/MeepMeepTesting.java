package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
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
                        drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(12.24, -29.60), Math.toRadians(86.57))
                                .splineTo(new Vector2d(47.70, -11.88), Math.toRadians(1.76))
                                .splineTo(new Vector2d(44.59, 36.00), Math.toRadians(183.79))
                                .splineTo(new Vector2d(4.20, 35.45), Math.toRadians(177.46))
                                .splineTo(new Vector2d(-16.08, 19.01), Math.toRadians(236.84))
                                .splineTo(new Vector2d(-33.62, 2.38), Math.toRadians(-90.00))
                                .splineTo(new Vector2d(-12.24, -23.76), Math.toRadians(-70.90))
                                .splineTo(new Vector2d(-12.43, -61.58), Math.toRadians(269.54))
                                .build()
                );

        // to speed up ,SampleMecanumDrive.getVelocityConstraint(90, 90, 13.24),SampleMecanumDrive.getAccelerationConstraint(90)
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setAxesInterval(20)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                //.addEntity(mySecondBot)
                .start();
    }
}