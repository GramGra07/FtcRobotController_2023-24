package org.firstinspires.ftc.teamcode.UtilClass.state_machine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class StateMachine<T extends Enum<T>> {
    List<T> states;
    Map<T, StateChangeCallback> onEnterCommands;
    Map<T, StateChangeCallback> onExitCommands;
    Map<T, Supplier<Boolean>> transitions;
    T currentState;
    List<T> stateHistory;
    Map<T, Supplier<Boolean>> stopConditions;
    private boolean isRunning = false;

    public boolean isRunning() {
        return isRunning;
    }

    StateMachine(Builder<T> builder) {
        this.states = builder.states;
        this.onEnterCommands = builder.onEnterCommands;
        this.onExitCommands = builder.onExitCommands;
        this.transitions = builder.transitions;
        this.currentState = null;
        this.stateHistory = new ArrayList<>();
        this.stopConditions = builder.stopConditions;
    }

    public T getCurrentState() {
        return currentState;
    }

    public static class Builder<T extends Enum<T>> {
        List<T> states;
        Map<T, StateChangeCallback> onEnterCommands;
        Map<T, StateChangeCallback> onExitCommands;
        Map<T, Supplier<Boolean>> transitions;
        Map<T, Supplier<Boolean>> stopConditions;

        public Builder() {
            states = new ArrayList<>();
            onEnterCommands = new HashMap<>();
            onExitCommands = new HashMap<>();
            transitions = new HashMap<>();
            stopConditions = new HashMap<>();
        }

        public Builder<T> state(T state) {
            states.add(state);
            return this;
        }

        public Builder<T> whileState(T state, Supplier<Boolean> escapeCondition) {
            while (!escapeCondition.get()) {
                onEnterCommands.get(state).onStateChange();
            }
            return this;
        }

        public Builder<T> onEnter(T state, StateChangeCallback command) {
            onEnterCommands.put(state, command);
            return this;
        }

        public Builder<T> onExit(T state, StateChangeCallback command) {
            onExitCommands.put(state, command);
            return this;
        }

        public Builder<T> transition(T state, Supplier<Boolean> condition) {
            transitions.put(state, condition);
            return this;
        }

        public Builder<T> stopRunning(T state, Supplier<Boolean> condition) {
            stopConditions.put(state, condition);
            return this;
        }

        public StateMachine<T> build() {
            return new StateMachine<>(this);
        }
    }

    public void start() {
        if (!states.isEmpty()) {
            currentState = states.get(0);
            StateChangeCallback onEnterAction = onEnterCommands.get(currentState);
            if (onEnterAction != null) {
                onEnterAction.onStateChange();
            }
        }
    }

    public boolean update() throws SMExceptions {
        if (!states.isEmpty()) {
            currentState = states.get(0);
            Supplier<Boolean> transitionCondition = transitions.get(currentState);
            if (transitionCondition != null && transitionCondition.get()) {
                // Get the next state
                T nextState = states.get(1); // assuming states has at least 2 elements
                // Check if the transition is valid
                if (!isValidTransition(currentState, nextState)) {
                    throw (new SMExceptions("Invalid transition"));
                }
                StateChangeCallback onExitAction = onExitCommands.get(currentState);
                if (onExitAction != null) {
                    onExitAction.onStateChange();
                }
                // Remove the current state
                states.remove(0);
                // If there are more states, enter the next one
                if (!states.isEmpty()) {
                    currentState = states.get(0);
                    StateChangeCallback onEnterAction = onEnterCommands.get(currentState);
                    if (onEnterAction != null) {
                        onEnterAction.onStateChange();
                    }
                }
            }
        }
        if (states.isEmpty()) {
            Supplier<Boolean> stopCondition = stopConditions.get(currentState);
            if (stopCondition != null && stopCondition.get()) {
                isRunning = false;
            }
        }
        return isRunning;
    }

    public boolean isValidTransition(T fromState, T toState) {
        // Check if 'fromState' is in 'states'
        if (!states.contains(fromState)) {
            return false;
        }

        // Check if 'toState' is a valid transition from 'fromState'
        Supplier<Boolean> transitionCondition = transitions.get(fromState);
        if (transitionCondition == null) {
            return false;
        }

        // If the transition condition is met, return true
        return transitionCondition.get();
    }

    public List<T> getStateHistory() {
        return stateHistory;
    }
}