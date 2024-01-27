package org.firstinspires.ftc.teamcode.UtilClass;

import static org.firstinspires.ftc.teamcode.UtilClass.StateMachineTest.state.STATE_two;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.place1Sort;
import static org.firstinspires.ftc.teamcode.opModes.autoSoftware.autoSorting.preselect;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import StateMachine.StateMachine;

@Autonomous(group = place1Sort, preselectTeleOp = preselect)
//@Disabled
public class StateMachineTest extends LinearOpMode {
    public enum state {
        INIT,
        STATE_one,
        STATE_two,
        STOP,
    }


    @Override
    public void runOpMode() {
        //initialization functions
        StateMachine<state> machine = new StateMachine.Builder<state>()
                .state(StateMachineTest.state.STATE_one)
                .onEnter(StateMachineTest.state.STATE_one, () -> {
                    // functions to run when entering this state
                })
                .whileState(StateMachineTest.state.STATE_one, () -> true) // condition to escape this state
                .transition(StateMachineTest.state.STATE_one, () -> true) // condition to transition to next state
                .state(STATE_two)
                .onEnter(STATE_two, () -> {
                    // functions to run when entering this state
                })
                .whileState(STATE_two, () -> true) // condition to escape this state
                .transition(STATE_two, () -> true) // condition to transition to next state
                .stopRunning() // condition to stop the state machine
                .build();
        waitForStart();
        machine.start(); // start the machine
        while (opModeIsActive() && machine.isRunning()) {
            machine.update(); // update the machine
        }
    }
}