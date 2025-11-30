package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.ServoWrapper;
import org.firstinspires.ftc.teamcode.util.LazyInit;

import java.util.function.Consumer;

public class TransferWheelController {
    ServoWrapper servo;

    LazyInit<IAction<Float>> transferPower;
    public IAction<Float> transferPowerAction() {return transferPower.get();}

    public TransferWheelController(ServoWrapper _servo) {
        servo = _servo;
        transferPower = new LazyInit<>(() -> IAction.From.loop(
                (robot, f) -> servo.setPosition((1 + f) / 2f)
        ));
    }

    public TransferWheelController(ServoWrapper _servo, Consumer<Boolean> _syncIntake) {
        servo = _servo;

        transferPower = new LazyInit<>(() -> IAction.From.loop(
                (robot, f) -> {
                    servo.setPosition((1 + f) / 2f);
                    _syncIntake.accept(f > 0); // THIS MAY BE INCORRECT -_-
                }
        ));
    }

}
