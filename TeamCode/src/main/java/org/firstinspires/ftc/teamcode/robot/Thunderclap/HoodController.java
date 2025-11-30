package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.IServo;
import org.firstinspires.ftc.teamcode.util.LazyInit;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

public class HoodController implements WithTelemetry.IWithTelemetry {
    IServo servo;
    float down,up;
    float pos;

    final LazyInit<IAction<Float>> setHoodPosition;
    public IAction<Float> setHoodPositionAction() {return setHoodPosition.get();}

    public HoodController(IServo _servo, float _down, float _up) {
        servo = _servo;
        down = _down;
        up = _up;
        pos = 0.5f;

        setHoodPosition = new LazyInit<>(() -> IAction.From.loop((r, f) ->
                {
                    pos = Util.clamp(pos - 0.09f * f, 0, 1);
                servo.setPosition(
                        (up - down) * pos + down
                );
                }));

        telemetryAction = new LazyInit<>(() -> WithTelemetry.fromLambda(() -> "Hood Controller", telemetry -> {
            telemetry.addData("Position", servo.getPosition());
        }));
    }

    final LazyInit<IAction<Telemetry>> telemetryAction;
    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction.get();
    }
}
