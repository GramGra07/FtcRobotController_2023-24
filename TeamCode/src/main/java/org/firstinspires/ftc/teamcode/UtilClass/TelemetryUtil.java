package org.firstinspires.ftc.teamcode.UtilClass;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryUtil {
    public static Telemetry createTelemetry(OpMode myOpMode) {
        return new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
    }
}
