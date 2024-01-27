## Builder Function

```
public static StateMachine<StateMachineTest.state> machine(MecanumDrive drive) {
StateMachine.Builder<StateMachineTest.state> builder = new StateMachine.Builder<>();
return builder
(your events)
}
```

## Requirements

```.stopRunning()```
after ```.transition(last, ()-> condition)```