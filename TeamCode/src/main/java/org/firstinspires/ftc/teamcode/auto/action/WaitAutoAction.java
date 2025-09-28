package org.firstinspires.ftc.teamcode.auto.action;

import org.firstinspires.ftc.teamcode.components.RobotController;
import org.firstinspires.ftc.teamcode.util.Pair;
import java.util.function.Function;

public class WaitAutoAction implements IAutoAction<ElapsedContainer>{

    final float duration;

    public WaitAutoAction(float _duration) {
        duration = _duration;
    }

    @Override
    public boolean isDone(float duration) {
        return duration >= this.duration;
    }

    @Override
    public Function<Pair<Float, RobotController>, ElapsedContainer> getDataProvider() {
        return _elapsed -> new ElapsedContainer(_elapsed.fst);
    }

    @Override
    public void init(RobotController robot, ElapsedContainer data) {

    }

    @Override
    public void start(RobotController robot, ElapsedContainer data) {

    }

    @Override
    public void loop(RobotController robot, ElapsedContainer data) {

    }
}
