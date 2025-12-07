package org.firstinspires.ftc.teamcode.auto.action;

import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.util.Pair;

import java.util.function.Function;

public class IActionAutoAction <T> implements IAutoAction<ElapsedContainer> {

    final float duration;
    final IAction<T> action;
    final Function<ElapsedContainer, T> dataProvider;

    public IActionAutoAction(float _duration, IAction<T> _action, Function<ElapsedContainer, T> _dataProvider) {
        duration = _duration;
        action = _action;
        dataProvider = _dataProvider;
    }

    @Override
    public boolean isDone(float duration) {
        return duration >= this.duration;
    }

    @Override
    public Function<Pair<Float, IRobot>, ElapsedContainer> getDataProvider() {
        return _elapsed -> new ElapsedContainer(_elapsed.fst);
    }

    @Override
    public void init(IRobot robot, ElapsedContainer data) {
        action.init(robot, dataProvider.apply(data));
    }

    @Override
    public void start(IRobot robot, ElapsedContainer data) {
        action.start(robot, dataProvider.apply(data));
    }

    @Override
    public void loop(IRobot robot, ElapsedContainer data) {
        action.loop(robot, dataProvider.apply(data));
    }
}

