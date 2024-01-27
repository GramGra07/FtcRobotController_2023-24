package org.firstinspires.ftc.teamcode.UtilClass.state_machine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
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
    private boolean isRunning = true;

    public <T extends Enum<T>> StateMachine(T initialState, java.util.Map onEnterCommands, java.util.Map onExitCommands, java.util.Map transitions, java.util.Map stopConditions) {
    }

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

    public void stopRunning() {
        this.isRunning = false;
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
        private StateMachine<T> machine;

        public Builder() {
            states = new ArrayList<>();
            onEnterCommands = new HashMap<>();
            onExitCommands = new HashMap<>();
            transitions = new HashMap<>();
            stopConditions = new HashMap<>();
        }

        public Builder<T> state(T state) {
            if (states.contains(state)) {
                throw new IllegalArgumentException("State already exists");
            }
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
            if (!states.contains(state)) {
                throw new IllegalArgumentException("State does not exist");
            }
            onEnterCommands.put(state, command);
            return this;
        }

        public Builder<T> onExit(T state, StateChangeCallback command) {
            if (!states.contains(state)) {
                throw new IllegalArgumentException("State does not exist");
            }
            onExitCommands.put(state, command);
            return this;
        }

        public Builder<T> transition(T state, Supplier<Boolean> condition) {
            if (!states.contains(state)) {
                throw new IllegalArgumentException("State does not exist");
            }
            transitions.put(state, condition);
            return this;
        }

        public Builder<T> stopRunning() {
            if (this.machine == null) {
                throw new IllegalStateException("StateMachine has not been built yet");
            }
            this.machine.stopRunning();
            return this;
        }

        public StateMachine<T> build() {
            if (states == null || states.isEmpty() || transitions == null || transitions.isEmpty()) {
                throw new IllegalArgumentException("States and transitions cannot be null or empty");
            }

            if (new HashSet<>(states).size() != states.size()) {
                throw new IllegalArgumentException("States cannot have duplicates");
            }

            if (states.size() != transitions.size()) {
                throw new IllegalArgumentException("Mismatched states and transitions");
            }

            for (T state : states) {
                if (!transitions.containsKey(state)) {
                    throw new IllegalArgumentException("All states must have a corresponding transition");
                }
                if (onEnterCommands.get(state) == null || onExitCommands.get(state) == null) {
                    throw new IllegalArgumentException("All states must have a corresponding onEnter and onExit command");
                }
                if (stopConditions.get(state) == null) {
                    throw new IllegalArgumentException("All states must have a corresponding stop condition");
                }
            }

            if (onEnterCommands.get(states.get(0)) == null) {
                throw new IllegalArgumentException("Initial state must have a corresponding onEnter command");
            }

            this.machine = new StateMachine<>(this);
            return this.machine;
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

    public boolean update() {
        if (!states.isEmpty()) {
            currentState = states.get(0);
            Supplier<Boolean> transitionCondition = transitions.get(currentState);
            if (transitionCondition != null && transitionCondition.get()) {
                // Get the next state
                T nextState = states.get(1); // assuming states has at least 2 elements
                // Check if the transition is valid
                if (!isValidTransition(currentState, nextState)) {
                    throw new IllegalStateException("Invalid transition");
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