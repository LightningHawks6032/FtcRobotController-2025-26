package org.firstinspires.ftc.teamcode.components.action;

import org.firstinspires.ftc.teamcode.components.IRobot;

import java.util.function.BiFunction;

/// `TySplit = conversionMap(Ty1, Ty2)`<br>
/// `leftSetter` - sets Ty1<br>
/// `rightSetter` - sets Ty2<br>
/// `this` - calls the `IAction<TySplit>``
public class SplitAction <Ty1, Ty2, TySplit> implements IAction<Object> {
    IAction<Ty1> leftSetter;
    IAction<Ty2> rightSetter;
    IAction<TySplit> action;

    public IAction<Ty1> leftSetter() {
        return leftSetter;
    }

    public IAction<Ty2> rightSetter() {
        return rightSetter;
    }

    Ty1 leftData;
    Ty2 rightData;

    TySplit last;

    BiFunction<Ty1, Ty2, TySplit> conversionMap;

    class LeftSetterAction implements IAction<Ty1> {

        @Override
        public void init(IRobot robot, Ty1 data) {
            leftData = data;
        }

        @Override
        public void start(IRobot robot, Ty1 data) {
            leftData = data;
        }

        @Override
        public void loop(IRobot robot, Ty1 data) {
            leftData = data;
        }
    }

    class RightSetterAction implements IAction<Ty2> {

        @Override
        public void init(IRobot robot, Ty2 data) {
            rightData = data;
        }

        @Override
        public void start(IRobot robot, Ty2 data) {
            rightData = data;
        }

        @Override
        public void loop(IRobot robot, Ty2 data) {
            rightData = data;
        }
    }

    public SplitAction(IAction<TySplit> _action, BiFunction<Ty1, Ty2, TySplit> _conversionMap) {
        action = _action;
        conversionMap = _conversionMap;
        leftSetter = new LeftSetterAction();
        rightSetter = new RightSetterAction();
    }

    @Override
    public void init(IRobot robot, Object data) {
        if (leftData != null && rightData != null) {
            last = conversionMap.apply(leftData, rightData);

            action.init(robot, last);
        }
    }

    @Override
    public void start(IRobot robot, Object data) {
        if (leftData != null && rightData != null) {
            last = conversionMap.apply(leftData, rightData);

            action.start(robot, last);
        }
    }

    @Override
    public void loop(IRobot robot, Object data) {
        if (leftData != null && rightData != null) {
            last = conversionMap.apply(leftData, rightData);

            action.loop(robot, last);
        }
    }

    public TySplit getLast() {return conversionMap.apply(leftData, rightData);}
}
