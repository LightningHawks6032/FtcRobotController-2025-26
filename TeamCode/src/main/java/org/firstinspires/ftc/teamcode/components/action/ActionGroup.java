package org.firstinspires.ftc.teamcode.components.action;

import org.firstinspires.ftc.teamcode.components.IRobot;

import java.util.ArrayList;
import java.util.Arrays;

public class ActionGroup <DataType> implements IAction<DataType> {
    final ArrayList<IAction<DataType>> actions;


    @SafeVarargs
    public ActionGroup(IAction<DataType>... _actions) {
        actions = new ArrayList<>(Arrays.asList(_actions));
    }

    public ActionGroup(ArrayList<IAction<DataType>> _actions) {
        actions = _actions;
    }

    @Override
    public void init(IRobot robot, DataType data) {
        for (IAction<DataType> action : actions) {
            action.init(robot, data);
        }
    }

    @Override
    public void start(IRobot robot, DataType data) {
        for (IAction<DataType> action : actions) {
            action.start(robot, data);
        }
    }

    @Override
    public void loop(IRobot robot, DataType data) {
        for (IAction<DataType> action : actions) {
            action.loop(robot, data);
        }
    }
}
