package org.firstinspires.ftc.teamcode.UtilClass;

import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.place1Sort;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.UtilClass.state_machine.exampleSm;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

@Autonomous(group = place1Sort, preselectTeleOp = preselect)
//@Disabled
public class StateMachineTest extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    public enum state {
        INIT,
        SPIKE_NAV,
        END_POSE,
        STOP,
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.LEFT));
        robot.initAuto(hardwareMap, this, false);
        waitForStart();
        StateMachine<state> machine = exampleSm.machine(drive);
        machine.start();
        while (opModeIsActive() && machine.isRunning()) {
            machine.update();
            drive.update();
        }
    }
}