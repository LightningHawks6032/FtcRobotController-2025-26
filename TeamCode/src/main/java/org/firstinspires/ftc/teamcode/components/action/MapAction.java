package org.firstinspires.ftc.teamcode.components.action;

import org.firstinspires.ftc.teamcode.components.IRobot;

import java.util.function.BiFunction;

/// `this` - [IAction<TyTo>] that applies a given mapping to convert one kind of action to another
public class MapAction <TyFrom, TyTo> implements IAction<TyTo> {
    IAction<TyFrom> action;
    BiFunction<IRobot, TyTo, TyFrom> map;

    public MapAction(IAction<TyFrom> _action, BiFunction<IRobot, TyTo, TyFrom> _map) {
        action = _action;
        map = _map;
    }

    @Override
    public void init(IRobot robot, TyTo data) {
        action.init(robot, map.apply(robot, data));
    }

    @Override
    public void start(IRobot robot, TyTo data) {
        action.start(robot, map.apply(robot, data));
    }

    @Override
    public void loop(IRobot robot, TyTo data) {
        action.loop(robot, map.apply(robot, data));
    }
}
