package org.firstinspires.ftc.teamcode.UtilClass;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Enums.AutoRandom;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;
import org.firstinspires.ftc.teamcode.opModes.autoHardware;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class HuskyLensUtil {
    public static com.qualcomm.hardware.dfrobot.HuskyLens huskyLens;
    public static final int READ_PERIOD = 1;
    public static Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
    public static List<Integer> blockInfo = new ArrayList<>();

    public static void initHuskyLens(HardwareMap hardwareMap, OpMode myOpMode, HuskyLens.Algorithm algorithm) {
        huskyLens = hardwareMap.get(com.qualcomm.hardware.dfrobot.HuskyLens.class, "huskylens");
        rateLimit.expire();
        if (!huskyLens.knock()) {
            myOpMode.telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            myOpMode.telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(algorithm);
    }

    public static void huskyLensTelemetry(Telemetry
                                                  thisTelemetry, boolean update) {
        if (rateLimit.hasExpired()) {
            rateLimit.reset();
        }
        rateLimit.reset();
        com.qualcomm.hardware.dfrobot.HuskyLens.Block[] blocks = huskyLens.blocks();
        thisTelemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            thisTelemetry.addData("Block", blocks[i].toString());
            if (blocks[i].id == 1) {
                blockInfo.removeAll(blockInfo);
                blockInfo.add(blocks[i].x);
                blockInfo.add(blocks[i].y);
                blockInfo.add(blocks[i].left);
                blockInfo.add(blocks[i].top);
                thisTelemetry.addData("", blockInfo);
            }
        }
        if (update) {
            thisTelemetry.update();
        }
    }

    public static void extrapolatePosition() {
        double cameraThird = 106.66666666666667;
        if (!blockInfo.isEmpty()) {
            if (blockInfo.get(0) < cameraThird) {
                autoHardware.autonomousRandom = AutoRandom.left;
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
            } else if (blockInfo.get(0) > cameraThird && blockInfo.get(0) < cameraThird * 2) {
                autoHardware.autonomousRandom = AutoRandom.mid;
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
            } else if (blockInfo.get(0) > cameraThird * 2) {
                autoHardware.autonomousRandom = AutoRandom.right;
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, true);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, true);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, true);
            } else {
                Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, false);
                Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
                Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
            }
        }
    }

    public static void delayIfNoOBJ(Telemetry telemetry) {
        while (blockInfo.size() == 0) {
            telemetry.addData("Nothing", "found");
            telemetry.update();
            HuskyLensUtil.huskyLensTelemetry(telemetry, true);
            Sensors.ledIND(HardwareConfig.green1, HardwareConfig.red1, false);
            Sensors.ledIND(HardwareConfig.green2, HardwareConfig.red2, false);
            Sensors.ledIND(HardwareConfig.green3, HardwareConfig.red3, false);
        }
    }
}
