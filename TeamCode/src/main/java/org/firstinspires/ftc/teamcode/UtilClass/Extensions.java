package org.firstinspires.ftc.teamcode.UtilClass;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Extensions {
    public Pose2d invertPose(Pose2d pose) {
        return new Pose2d(-pose.getY(), -pose.getX(), -pose.getHeading());
    }
}
