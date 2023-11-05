package org.firstinspires.ftc.teamcode.UtilClass;

import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.useFileWriter;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;

public class FileWriterFTC {
    public static final String file = String.format("%s/FIRST/matchlogs/log.txt", Environment.getExternalStorageDirectory().getAbsolutePath());

    public static void setUpFile(FileWriter fileWriter) {
        if (useFileWriter) {
            File myObj = new File(file);
            try {
                myObj.delete();
                myObj.createNewFile();
            } catch (Exception e) {
                e.printStackTrace();
            }
            try {
                fileWriter.write("");
                fileWriter.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public static void writeToFile(FileWriter fileWriter, int x, int y) {
        //  terminal
        //  clear
        //  adb shell
        //  cat /storage/emulated/0/FIRST/matchlogs/log.txt
        //  copy everything
        //  paste into file.txt
        //  run testGraphing in pycharm or other python IDE
        //  look at results in graph.png
        if (useFileWriter) {
            try {
                fileWriter = new FileWriter(file, true);
                fileWriter.append(String.valueOf(x)).append(" ").append(String.valueOf(y)).append(" \n");
                fileWriter.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
