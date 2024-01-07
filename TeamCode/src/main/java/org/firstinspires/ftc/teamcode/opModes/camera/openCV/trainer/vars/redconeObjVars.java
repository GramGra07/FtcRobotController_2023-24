package org.firstinspires.ftc.teamcode.opModes.camera.openCV.trainer.vars;

//@Config
public class redconeObjVars {
    public static double aspectRatio = 0.7872340425531915;
    public static double minWidth = 43.0;
    public static double minHeight = 58.0;
    public static double maxWidth = 108.0;
    public static double maxHeight = 145.0;
    public static double minArea = minWidth * minHeight;
    public static double maxArea = maxWidth * maxHeight;
    public static double tolerance = 0.2; // this is the value that will determine how far off the aspect ratio can be to still detect it, you will need to tune it more
    public static double translationX = 0.05357142857142857;
    public static double translationY = 0.5;
}