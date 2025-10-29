package org.firstinspires.ftc.teamcode.components.action;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.util.Pair;

/// Launches an [AutoActionSequence] <br> On `true` loop, the auto sequence will start
/// and will continue until termination <br>
/// A following `true` loop will initiate the sequence again<br>
public class LaunchAutoSequenceAction <DataType extends ElapsedContainer> implements IAction<Boolean> {
    AutoActionSequence<DataType> sequence;
    boolean running;
    ElapsedTime timer;

    public boolean running() {return running;}

    public LaunchAutoSequenceAction(AutoActionSequence<DataType> _sequence) {
        sequence = _sequence;
        running = false;
        timer = new ElapsedTime();
    }


    @Override
    public void init(IRobot robot, Boolean data) {
        sequence.init(robot, sequence.getDataProvider().apply(new Pair<>((float) timer.seconds(), robot)));
    }

    @Override
    public void start(IRobot robot, Boolean data) {
        sequence.start(robot, sequence.getDataProvider().apply(new Pair<>((float) timer.seconds(), robot)));
        timer.reset();
    }

    @Override
    public void loop(IRobot robot, Boolean data) {
        if (!running && data) {
            running = true;
            timer.reset();
            sequence.reset();
        }

        if (running) {
            sequence.loop(robot, sequence.getDataProvider().apply(new Pair<>((float) timer.seconds(), robot)));

            if (sequence.isDone((float) timer.seconds())) {
                running = false;
            }
        }
    }
}
