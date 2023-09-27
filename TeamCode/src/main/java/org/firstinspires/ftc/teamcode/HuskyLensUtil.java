package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class HuskyLensUtil {
    public static com.qualcomm.hardware.dfrobot.HuskyLens huskyLens;
    public static final int READ_PERIOD = 1;
    public static Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
    public static void initHuskyLens(HardwareMap hardwareMap, OpMode myOpMode, HuskyLens.Algorithm algorithm){
        huskyLens = hardwareMap.get(com.qualcomm.hardware.dfrobot.HuskyLens.class, "huskylens");
        rateLimit.expire();
        if (!huskyLens.knock()) {
            myOpMode.telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            myOpMode.telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(algorithm);
    }
    public static void huskyLensTelemetry(OpMode myOpMode, boolean update){
        if (rateLimit.hasExpired()) {
            rateLimit.reset();
        }
        rateLimit.reset();
        com.qualcomm.hardware.dfrobot.HuskyLens.Block[] blocks = huskyLens.blocks();
        myOpMode.telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            myOpMode.telemetry.addData("Block", blocks[i].toString());
        }
        if (update){myOpMode.telemetry.update();}
    }
}
