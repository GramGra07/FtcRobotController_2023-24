package org.firstinspires.ftc.teamcode.UtilClass.state_machine;

import static org.firstinspires.ftc.teamcode.UtilClass.state_machine.testStatemachine.STATE.INIT;
import static org.firstinspires.ftc.teamcode.UtilClass.state_machine.testStatemachine.STATE.STOP;
import static org.firstinspires.ftc.teamcode.opModes.HardwareConfig.flipServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.UtilClass.ServoUtil;
import org.firstinspires.ftc.teamcode.opModes.HardwareConfig;

@TeleOp
//@Disabled
public class testStatemachine extends LinearOpMode {
    HardwareConfig robot = new HardwareConfig(this);


    public enum STATE {
        INIT,
        STOP,
    }

    StateMachine<STATE> machine = new StateMachine.Builder<STATE>()
            .state(INIT)
            .onEnter(INIT, () -> {
                ServoUtil.calculateFlipPose(30, flipServo);
            })
            .transition(INIT, () -> gamepad1.square)
            .state(STOP)
            .onEnter(STOP, () -> {
                ServoUtil.calculateFlipPose(0, flipServo);
            })
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, false);
//        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        waitForStart();
        machine.start();
        while (opModeIsActive()) {
            try {
                machine.update();
            } catch (SMExceptions e) {
                throw new RuntimeException(e);
            }

        }
    }
}