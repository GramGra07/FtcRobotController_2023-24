package org.firstinspires.ftc.teamcode.UtilClass.state_machine;

import static org.firstinspires.ftc.teamcode.UtilClass.state_machine.StateMachineTest.state.END_POSE;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.drive;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware.getStartPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.place1Sort;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.endPose.goToEndPose;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.generalPatterns.SpikeNav;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.EndPose;
import org.firstinspires.ftc.teamcode.Enums.PathLong;
import org.firstinspires.ftc.teamcode.Enums.StartSide;
import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoHardware;
import org.firstinspires.ftc.teamcode.opModes.rr.drive.MecanumDrive;

@Autonomous(group = place1Sort, preselectTeleOp = preselect)
@Disabled
public class StateMachineTest extends LinearOpMode {
    public Pose2d startPose = autoHardware.startPose;
    autoHardware robot = new autoHardware(this);

    public enum state {
        SPIKE_NAV,
        END_POSE,
        STOP,
        INIT,
    }

    StateMachine<state> machine = new StateMachine.Builder<state>()
//            .state(state.INIT)
            .state(state.SPIKE_NAV)
            .onEnter(state.SPIKE_NAV, () -> {
                ServoUtil.calculateFlipPose(0, flipServo);
                SpikeNav(drive, PathLong.NONE);
                drive.update();
            })
            .transition(state.SPIKE_NAV, () -> !drive.isBusy())
            .state(END_POSE)
            .onEnter(END_POSE, () -> {
                goToEndPose(EndPose.StartingPosition, drive);
                drive.update();
            })
            .transition(END_POSE, () -> !drive.isBusy())
            .state(state.STOP)
            .onEnter(state.STOP, () -> {
            })
            .build();

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose(Alliance.BLUE, StartSide.LEFT));
        robot.initAuto(hardwareMap, this, false);
        machine.start();
        while (opModeIsActive()) {
            try {
                machine.update();
                drive.update();
            } catch (SMExceptions e) {
                throw new RuntimeException(e);
            }
        }
    }
}