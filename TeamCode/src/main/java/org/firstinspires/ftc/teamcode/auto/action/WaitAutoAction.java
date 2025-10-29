package org.firstinspires.ftc.teamcode.auto.action;

import org.firstinspires.ftc.teamcode.components.IRobot;
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
    public Function<Pair<Float, IRobot>, ElapsedContainer> getDataProvider() {
        return _elapsed -> new ElapsedContainer(_elapsed.fst);
    }

    @Override
    public void init(IRobot robot, ElapsedContainer data) {

    }

    @Override
    public void start(IRobot robot, ElapsedContainer data) {

    }

    @Override
    public void loop(IRobot robot, ElapsedContainer data) {

    }
}
