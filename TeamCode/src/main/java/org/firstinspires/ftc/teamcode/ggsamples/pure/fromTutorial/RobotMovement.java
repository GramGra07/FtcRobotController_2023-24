package org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial;

import static org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.RobotUtilities.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.RobotUtilities.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.RobotUtilities.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.com.company.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.com.company.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.com.company.Robot.worldYPosition;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ggsamples.pure.fromTutorial.org.opencv.core.Point;

import java.util.ArrayList;

public class RobotMovement extends MyOpMode {

    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition), allPoints.get(0).followDistance);
        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for (int i = 0; i < pathPoints.size() - 1; i++) {
            if (i >= pathPoints.size() - 1) break; //added this to prevent out of bounds error
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = MathFunctions.LineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());
            double closestAngle = 10000000;
            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));
                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
        if (distanceToTarget < 10) {
            movement_turn = 0;
        }
        double yControl = movement_y;
        double xControl = movement_x;
        double turn = movement_turn;
        double frontRightPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) + turn);
        double backRightPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) + turn);
        double frontLeftPower = (yControl * Math.abs(yControl) + xControl * Math.abs(xControl) - turn);
        double backLeftPower = (yControl * Math.abs(yControl) - xControl * Math.abs(xControl) - turn);
        applyPower(frontRightPower, backRightPower, frontLeftPower, backLeftPower);
    }
}
