package org.firstinspires.ftc.teamcode.UtilClass;

import org.opencv.core.Scalar;

public class ScalarUtil {
    // color map below
    // https://i.stack.imgur.com/gyuw4.png

    public static Scalar fetchScalar(String type, String name, int num) {
        switch (type) {
            case "l":
                //low
                switch (name) {
                    case "red":
                        switch (num) {
                            case 1:
                                return new Scalar(0, 70, 50);
                            case 2:
                                return new Scalar(172, 70, 50);
                        }
                    case "yellow":
                        return new Scalar(20, 100, 100);
                    case "blue":
                        return new Scalar(100, 100, 100);
                    case "white":
                        return new Scalar(0, 0, 100);
                    case "green":
                        return new Scalar(40, 100, 100);
                }

            case "h":
                //high
                switch (name) {
                    case "red":
                        switch (num) {
                            case 1:
                                return new Scalar(8, 255, 255);
                            case 2:
                                return new Scalar(180, 255, 255);
                        }
                    case "yellow":
                        return new Scalar(30, 255, 255);
                    case "blue":
                        return new Scalar(140, 255, 255);
                    case "white":
                        return new Scalar(180, 255, 255);
                    case "green":
                        return new Scalar(75, 255, 255);
                }
        }
        return new Scalar(0, 0, 0);
    }

    public static Scalar scalarVals(String color) {//rgb scalar vals
        switch (color) {
            case "red":
                color = "red";
                break;
            case "blue":
                color = "blue";
                break;
            case "yellow":
                color = "yellow";
                break;
            case "green":
                color = "green";
                break;
            case "white":
                color = "white";
                break;
        }

        if (color == "yellow") {
            return new Scalar(255, 255, 0);
        } else if (color == "blue") {
            return new Scalar(0, 0, 255);
        } else if (color == "green") {
            return new Scalar(0, 255, 0);
        } else if (color == "red") {
            return new Scalar(255, 0, 0);
        } else if (color == "black") {
            return new Scalar(0, 0, 0);
        } else if (color == "white") {
            return new Scalar(255, 255, 255);
        } else {
            return new Scalar(255, 255, 255);
        }
    }
}
