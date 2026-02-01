package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IActionAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.WaitAutoAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.hardware.ServoWrapper;
import org.firstinspires.ftc.teamcode.util.Toggle;

public class KickerController {
    ServoWrapper servo;
    Toggle toggle;
    LaunchAutoSequenceAction<ElapsedContainer> kickerAction;

    public KickerController(ServoWrapper _servo) {
        toggle = new Toggle();
        servo = _servo;

        kickerAction = new LaunchAutoSequenceAction<>(
                new AutoActionSequence<>(
                        new IActionAutoAction<>(0.3f, IAction.From.loop((robot, t) -> servo.setPosition(0.7f)), it -> true),
                        new IActionAutoAction<>(0.1f, IAction.From.loop((robot, t) -> servo.setPosition(0.31f)), it -> true),
                        new WaitAutoAction(1f)
                        //new IActionAutoAction<>(0.1f, IAction.From.loop((robot, t) -> spindexerCommander.setNearestBallState(spindexerController.getMotorPos(), SpindexerController.BallState.NONE)), it -> true)
                )
        );

    }

    public IAction<Boolean> getKickerAction() {
        return kickerAction;
    }

    public boolean isRunning() {return kickerAction.running();}



}
