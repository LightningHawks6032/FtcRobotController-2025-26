package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.ServoWrapper;
import org.firstinspires.ftc.teamcode.util.LazyInit;
import org.firstinspires.ftc.teamcode.util.Toggle;

public class IntakeSwingController {
    ServoWrapper servo;
    Toggle toggle;

    final static float up = 0.9f;
    final static float down = 0.2f;

    LazyInit<IAction<Boolean>> swingIntake;
    public IAction<Boolean> getSwingIntakeAction() {
        return swingIntake.get();
    }

    LazyInit<IAction<Boolean>> assertSwing;
    public IAction<Boolean> assertSwingAction() {return assertSwing.get();}

    public IntakeSwingController(ServoWrapper _servo) {
        servo = _servo;
        toggle = new Toggle();

        swingIntake = new LazyInit<>(() -> IAction.From.loop((r, b) -> {
            toggle.loop(b);
            servo.setPosition(toggle.toggle() ? up : down);
        }));

        assertSwing = new LazyInit<>(() -> IAction.From.loop((r, b) -> {
            toggle.setToggle(b);
        }));
    }



}
