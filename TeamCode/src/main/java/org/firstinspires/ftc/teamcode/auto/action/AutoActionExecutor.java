package org.firstinspires.ftc.teamcode.auto.action;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.RobotController;

public class AutoActionExecutor<DataType extends ElapsedContainer> {
    final ElapsedTime timer;
    final IAutoAction<DataType> action;
    boolean finished;

    public AutoActionExecutor(IAutoAction<DataType> _action) {
        timer = new ElapsedTime();
        action = _action;
        finished = false;
    }

    public void init(RobotController robot) {
        action.init(robot, action.getDataProvider().apply(0f));
    }

    public void start(RobotController robot) {
        action.start(robot, action.getDataProvider().apply(0f));
        timer.reset();
    }

    public void loop(RobotController robot) {
        if (action.isDone((float) timer.seconds())) {
            finished = true;
        }

        if (!finished) {
            action.loop(robot, action.getDataProvider().apply((float) timer.seconds()));
        }
    }
}
