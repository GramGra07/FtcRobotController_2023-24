package org.firstinspires.ftc.teamcode.UtilClass.varStorage;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class PIDVals {
    public static PIDFCoefficients extensionPIDFCo = new PIDFCoefficients(0.01, 0, 0, 0);
    public static PIDFCoefficients rotationPIDFCo = new PIDFCoefficients(0.0005, 0, 0, 0.0001);

}
