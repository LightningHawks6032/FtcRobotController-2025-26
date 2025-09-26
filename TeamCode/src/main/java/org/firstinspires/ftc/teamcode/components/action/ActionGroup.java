package org.firstinspires.ftc.teamcode.components.action;

import org.firstinspires.ftc.teamcode.components.RobotController;

import java.util.ArrayList;
import java.util.Arrays;

public class ActionGroup <DataType> implements IAction<DataType> {
    final ArrayList<IAction<DataType>> actions;


    @SafeVarargs
    public ActionGroup(IAction<DataType>... _actions) {
        actions = new ArrayList<>(Arrays.asList(_actions));
    }

    @Override
    public void init(RobotController robot, DataType data) {
        for (IAction<DataType> action : actions) {
            action.init(robot, data);
        }
    }

    @Override
    public void start(RobotController robot, DataType data) {
        for (IAction<DataType> action : actions) {
            action.start(robot, data);
        }
    }

    @Override
    public void loop(RobotController robot, DataType data) {
        for (IAction<DataType> action : actions) {
            action.loop(robot, data);
        }
    }
}
