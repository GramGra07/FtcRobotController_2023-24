package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.externalHardware.HardwareConfig.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.externalHardware.HardwareConfig;

@Autonomous(name = "PurePursuitSample", group = "Robot")
@Disabled
public class PurePursuitSample extends CommandOpMode {

    // define our constants
    static final double TRACKWIDTH = 13;
    static final double WHEEL_DIAMETER = 3.89;    // inches
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = 2.4;

    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private PurePursuitCommand ppCommand;
    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    private MotorEx leftEncoder, rightEncoder, centerEncoder;
    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void initialize() {
        robot.init(hardwareMap);
        fL = new Motor(hardwareMap, "motorFrontLeft");
        fR = new Motor(hardwareMap, "motorFrontRight");
        bL = new Motor(hardwareMap, "motorBackLeft");
        bR = new Motor(hardwareMap, "motorBackRight");

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);

        bL.setInverted(true);
        fL.setInverted(true);

        leftEncoder = new MotorEx(hardwareMap, "motorBackLeft");
        rightEncoder = new MotorEx(hardwareMap, "motorBackRight");
        centerEncoder = new MotorEx(hardwareMap, "deadWheel");

        // calculate multiplier
        TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / 8192;
        double TICKS_TO_INCHES_POWERED = WHEEL_DIAMETER * Math.PI / 423.2116;
        // create our odometry object and subsystem
        m_robotOdometry = new HolonomicOdometry(
                () -> leftEncoder.getCurrentPosition() * TICKS_TO_INCHES_POWERED,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES_POWERED,
                () -> centerEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        OdometrySubsystem m_odometry = new OdometrySubsystem(m_robotOdometry);
        Pose2d pose2d = m_odometry.getPose();

        //
        // With a preferred angle.
        //Waypoint p2 = new GeneralWaypoint(
        //        x, y, rotationRadians,
        //        movementSpeed, turnSpeed,
        //        followRadius
        //);
        double turnSpeed = 0.8;
        double moveSpeed = 0.8;
        double rotationBuffer = 10;
        double followRadius = 10;
        Waypoint p1 = new StartWaypoint(pose2d);
        Waypoint p2 = new GeneralWaypoint(1, 0, moveSpeed, turnSpeed, 10);
        Waypoint p3 = new GeneralWaypoint(1, 3);
        Waypoint p4 = new GeneralWaypoint(2, 3);
        Waypoint p5 = new GeneralWaypoint(2, 3.6, 90,
                moveSpeed, turnSpeed, followRadius);
        Waypoint p6 = new PointTurnWaypoint(
                3, 3, 90, moveSpeed,
                turnSpeed, followRadius, 2, rotationBuffer);
        //Waypoint p7 = new InterruptWaypoint(
        //        3, 2, moveSpeed,
        //        turnSpeed, followRadius, 2, rotationBuffer, this::runArmTop);//, robot.armEncoder(robot.topPoleVal,1,2,false));
        Waypoint p8 = new PointTurnWaypoint(
                3, 2, -90, moveSpeed,
                turnSpeed, followRadius,
                2, rotationBuffer
        );
        Waypoint p9 = new EndWaypoint();
        Path m_path = new Path(p1, p2, p3, p4, p5, p6, p8, p9);
        m_path.init();
        m_path.followPath(m_robotDrive, m_robotOdometry);
        // create our pure pursuit command
        //PurePursuitCommand ppCommand = new PurePursuitCommand(
        //        m_robotDrive, m_odometry,
        //        p1, p2, p3, p4, p5, p6, p8, p9
        //);
        //ppCommand.schedule();
    }
}