package org.firstinspires.ftc.teamcode.auto.action;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.RobotController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Function;

public class AutoActionSequence <DataType extends ElapsedContainer> implements IAutoAction<DataType>{
    int idx = 0;
    final ElapsedTime timer;
    final ArrayList<IAutoAction<DataType>> actions;

    public AutoActionSequence() {
        actions = new ArrayList<>();
        timer = new ElapsedTime();
    }

    @SafeVarargs
    public AutoActionSequence(IAutoAction<DataType>... _actions) {
        this();
        actions.addAll(Arrays.asList(_actions));
    }

    @Override
    public Function<Float, DataType> getDataProvider() {
        return actions.get(idx).getDataProvider();
    }

    @Override
    public void init(RobotController robot, DataType data) {
        for (IAutoAction<DataType> action : actions) {
            action.init(robot, action.getDataProvider().apply(data.elapsed));
        }
    }

    @Override
    public void start(RobotController robot, DataType data) {
        for (IAutoAction<DataType> action : actions) {
            action.start(robot, action.getDataProvider().apply(data.elapsed));
        }
    }

    @Override
    public void loop(RobotController robot, DataType data) {
        if (idx >= actions.size()) {return;}

        actions.get(idx).loop(robot, data);

        if (actions.get(idx).isDone((float) timer.seconds())) {
            idx++;
            timer.reset();
        }



    }

    @Override
    public boolean isDone(float duration) {
        return idx >= actions.size() ||
                actions.get(idx).isDone((float) timer.seconds());
    }

    public void reset() {
        idx = 0;
        timer.reset();
    }
}
