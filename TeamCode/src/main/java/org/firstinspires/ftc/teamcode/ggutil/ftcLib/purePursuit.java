package org.firstinspires.ftc.teamcode.ggutil.ftcLib;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

@Autonomous
@Disabled
public class purePursuit extends CommandOpMode {

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
        fL = new Motor(hardwareMap, "motorFrontLeft");
        fR = new Motor(hardwareMap, "motorFrontRight");
        bL = new Motor(hardwareMap, "motorBackLeft");
        bR = new Motor(hardwareMap, "motorBackRight");
        bL.setInverted(true);

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);

        leftEncoder = new MotorEx(hardwareMap, "motorBackLeft");
        rightEncoder = new MotorEx(hardwareMap, "motorBackRight");
        //centerEncoder = new MotorEx(hardwareMap, "motorFrontRight");

        // calculate multiplier
        double GEAR_RATIO = 1 / (5.23 * 2.89);
        double TICKS_PER_REV = 423.2116 / 4;
        double WHEEL_DIAMETER = 3.89;
        double PulsePR = (GEAR_RATIO * TICKS_PER_REV);
        TICKS_TO_INCHES = PulsePR / ((PI * WHEEL_DIAMETER) * 2 * PI);
        double TICKS_TO_INCHES_POWERED = WHEEL_DIAMETER * PI / 423.2116;
        // create our odometry object and subsystem
        //m_robotOdometry = new HolonomicOdometry(
        //        () -> leftEncoder.getCurrentPosition() * TICKS_TO_INCHES,
        //        () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES,
        //        () -> centerEncoder.getCurrentPosition() * TICKS_TO_INCHES,
        //        TRACKWIDTH, CENTER_WHEEL_OFFSET
        //);
        DifferentialOdometry m_robotOdometry = new DifferentialOdometry(
                () -> leftEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                TRACKWIDTH
        );
        OdometrySubsystem m_odometry = new OdometrySubsystem(m_robotOdometry);

        Pose2d current_pos = m_odometry.getPose();

        Translation2d endP = new Translation2d(0, 700);
        Rotation2d endR = new Rotation2d(0);
        Pose2d endPose = new Pose2d(endP, endR);

        double turnSpeed = 0.8;
        double moveSpeed = 0.8;
        double rotationBuffer = 1.5;
        double followRadius = 30;

        Waypoint p1 = new StartWaypoint(0, 0);
        Waypoint p2 = new PointTurnWaypoint(
                0, 500, 0, moveSpeed,
                turnSpeed, followRadius, 1, rotationBuffer);
        Waypoint p9 = new EndWaypoint(endP, endR, moveSpeed, turnSpeed, followRadius, 10, rotationBuffer);
        //Path m_path = new Path(p1,p2, p9);//, p3, p4, p5, p6, p8, p9);
        //m_path.init();
        //m_path.disableRetrace();
        //m_path.followPath(m_robotDrive, m_robotOdometry);
        // create our pure pursuit command
        ppCommand = new PurePursuitCommand(
                m_robotDrive, m_odometry,
                p1, p2, p9
        );
        ppCommand.schedule();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        ppCommand.execute();
    }
}