package org.firstinspires.ftc.teamcode.auto.action;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.util.Pair;

public class AutoActionExecutor<DataType extends ElapsedContainer> {
    final ElapsedTime timer;
    final IAutoAction<DataType> action;
    boolean finished;

    public AutoActionExecutor(IAutoAction<DataType> _action) {
        timer = new ElapsedTime();
        action = _action;
        finished = false;
    }

    public void init(IRobot robot) {
        action.init(robot, action.getDataProvider().apply(new Pair<>(0f, robot)));
    }

    public void start(IRobot robot) {
        action.start(robot, action.getDataProvider().apply(new Pair<>(0f, robot)));
        timer.reset();
    }

    public void loop(IRobot robot) {
        if (action.isDone((float) timer.seconds())) {
            finished = true;
        }

        if (!finished) {
            action.loop(robot, action.getDataProvider().apply(new Pair<>((float) timer.seconds(), robot)));
        }
    }
}
