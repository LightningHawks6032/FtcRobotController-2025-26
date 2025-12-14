package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.ToggleAction;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.util.LazyInit;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

public class IntakeWheelController implements WithTelemetry.IWithTelemetry {
    enum STATE {
        OFF,IN,EJECT,SYNCTRANSFER
    }

    DcMotorWrapper motor;
    STATE state;
    STATE lastState;

    LazyInit<IAction<Boolean>> motorPowerToggle;
    public IAction<Boolean> motorPowerToggleAction() {return motorPowerToggle.get();}

    LazyInit<IAction<Boolean>> ejectPower;
    public IAction<Boolean> ejectPowerAction() {return ejectPower.get();}

    /// This function sound pretty confusing ngl...
    /// When the transfer moves the artifact up the intake should sync with it
    /// This should not supersede ejection
    /// `up` indicates whether the ball is moving up **in the transfer**
    public void trySyncTransferLift(boolean up) {
        if (up && (state == STATE.IN || state == STATE.OFF)) {
            state = STATE.SYNCTRANSFER;
            motor.setPower(1f);
        }
        else if (!up && state == STATE.SYNCTRANSFER) {
            if (lastState == STATE.IN) {
                motor.setPower(1f);
                state = lastState;
            }
            else if (lastState == STATE.OFF) {
                motor.setPower(0f);
                state = lastState;
            }
        }
    }

    public IntakeWheelController(DcMotorWrapper _motor) {
        motor = _motor;
        state = STATE.OFF;
        lastState = state;

        motorPowerToggle = new LazyInit<>(() -> new ToggleAction().build(
            IAction.From.loop((r, b) -> {
                if (b && state == STATE.OFF) {
                    motor.setPower(1f);
                    state = STATE.IN;
                    lastState = state;
                }
                else if (!b && state == STATE.IN) {
                    motor.setPower(0f);
                    state = STATE.OFF;
                    lastState = state;
                }
            })
        ));

        ejectPower = new LazyInit<>(() -> IAction.From.loop(
                (r, b) -> {
                    if (b) {
                        motor.setPower(-1);
                        state = STATE.EJECT;
                    }
                    else if (state == STATE.EJECT) {
                        motor.setPower(0f);
                        state = STATE.OFF;
                        lastState = state;
                    }
                }
        ));

        telemetryAction = new LazyInit<>(() -> WithTelemetry.fromLambda(
                () -> "Intake Wheel Controller",
                telem -> {
                    telem.addData("State", state.toString());
                    telem.addData("Motor power", motor.getPower());
                }
        ));
    }

    LazyInit<IAction<Telemetry>> telemetryAction;

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction.get();
    }
}
