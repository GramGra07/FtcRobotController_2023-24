package org.firstinspires.ftc.teamcode.opModes.camera.openCV;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ConeObjVars {
    public static double aspectRatio = 0.69;
    public static double minWidth = 25;
    public static double minHeight = 40;
    public static double maxWidth = 100;
    public static double maxHeight = 150;
    public static double minArea = 1000;
    public static double maxArea = 10000;
    public static double tolerance = 0.1;

    public static double translationX = 0.0;
    public static double translationY = 0.6;

    // distance away from cam forward , bottom to the bottom of the cone
    // distance left right is from the center of the cone to middle of input
    // will report distance in both directions away from the camera

}
