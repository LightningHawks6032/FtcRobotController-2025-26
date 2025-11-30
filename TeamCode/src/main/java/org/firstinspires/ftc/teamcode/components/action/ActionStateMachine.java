package org.firstinspires.ftc.teamcode.components.action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.util.LazyInit;
import org.firstinspires.ftc.teamcode.util.StateMachine;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

import java.util.function.Function;

public class ActionStateMachine<State, Ty> extends StateMachine<State> implements IAction<Ty>, WithTelemetry.IWithTelemetry {

    Function<State, IAction<Ty>> map;

    @Override
    public void init(IRobot robot, Ty data) {
        map.apply(getCurrentNode()).init(robot, data);
    }

    @Override
    public void start(IRobot robot, Ty data) {
        map.apply(getCurrentNode()).start(robot, data);
    }

    @Override
    public void loop(IRobot robot, Ty data) {
        loop();
        map.apply(getCurrentNode()).loop(robot, data);
    }

    public ActionStateMachine(Function<State, IAction<Ty>> _map) {
        super();
        map = _map;

        telemetryAction = new LazyInit<>(() -> WithTelemetry.fromLambda(() -> "State Machine", telemetry ->
                telemetry.addData("State", getCurrentNode().toString())));
    }

    LazyInit<IAction<Telemetry>> telemetryAction;

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction.get();
    }
}
