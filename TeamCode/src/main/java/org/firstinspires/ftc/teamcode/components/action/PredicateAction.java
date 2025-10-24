package org.firstinspires.ftc.teamcode.components.action;

import org.firstinspires.ftc.teamcode.components.RobotController;

import java.util.function.BiFunction;

public class PredicateAction <Ty> implements IAction<Ty> {

    IAction<Ty> onTrue;
    IAction<Ty> onFalse;
    BiFunction<RobotController, Ty, Boolean> pred;


    public PredicateAction(IAction<Ty> _onTrue) {
        onTrue = _onTrue;
        onFalse = new EmptyAction<>();
    }

    public PredicateAction(IAction<Ty> _onTrue, IAction<Ty> _onFalse, BiFunction<RobotController, Ty, Boolean> _pred) {
        onTrue = _onTrue;
        onFalse = _onFalse;
        pred = _pred;
    }

    IAction<Ty> getRunningAction(boolean pred) {
        return pred ? onTrue : onFalse;
    }

    @Override
    public void init(RobotController robot, Ty data) {
        getRunningAction(pred.apply(robot, data)).init(robot, data);
    }

    @Override
    public void start(RobotController robot, Ty data) {
        getRunningAction(pred.apply(robot, data)).start(robot, data);
    }

    @Override
    public void loop(RobotController robot, Ty data) {
        getRunningAction(pred.apply(robot, data)).loop(robot, data);
    }
}
