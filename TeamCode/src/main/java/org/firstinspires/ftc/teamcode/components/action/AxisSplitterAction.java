package org.firstinspires.ftc.teamcode.components.action;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.jetbrains.annotations.Contract;

public class AxisSplitterAction {
    @NonNull
    @Contract("_, _ -> new")
    public static IAction<Vec2> TwoWay(IAction<Float> horizontal, IAction<Float> vertical) {
        return new IAction<Vec2>(){

            @Override
            public void init(IRobot robot, Vec2 data) {
                horizontal.init(robot, data.x);
                vertical.init(robot, data.y);
            }

            @Override
            public void start(IRobot robot, Vec2 data) {
                horizontal.start(robot, data.x);
                vertical.start(robot, data.y);
            }

            @Override
            public void loop(IRobot robot, Vec2 data) {
                horizontal.loop(robot, data.x);
                vertical.loop(robot, data.y);
            }
        };
    }

    @NonNull
    @Contract("_, _ -> new")
    public static IAction<Float> Axis(IAction<Boolean> pos, IAction<Boolean> neg) {
        return new IAction<Float>() {
            @Override
            public void init(IRobot robot, Float data) {
                pos.init(robot, data > 0);
                neg.init(robot, data < 0);
            }

            @Override
            public void start(IRobot robot, Float data) {
                pos.start(robot, data > 0);
                neg.start(robot, data < 0);
            }

            @Override
            public void loop(IRobot robot, Float data) {
                pos.loop(robot, data > 0);
                neg.loop(robot, data < 0);
            }
        };
    }

    @NonNull
    @Contract("_, _, _, _ -> new")
    public static IAction<Vec2> FourWay(IAction<Boolean> up,
                                        IAction<Boolean> right,
                                        IAction<Boolean> down,
                                        IAction<Boolean> left) {
        return TwoWay(
                Axis(right, left),
                Axis(up, down)
        );
    }


}
