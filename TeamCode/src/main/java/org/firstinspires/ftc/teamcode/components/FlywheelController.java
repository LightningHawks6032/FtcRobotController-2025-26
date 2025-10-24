package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.util.Toggle;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

/// `setPowerAction` - Sets the internal motor power<br>
/// `setLockAction` - Locks the internal motor power<br>
/// `setMotorPowerAction` - Sets the physical motor power
public class FlywheelController implements WithTelemetry.IWithTelemetry {
    IMotor motor;
    float power;
    boolean lock;

    final SetPowerAction setPowerAction;
    final SetLockAction setLockAction;
    final SetMotorPowerAction setMotorPowerAction;

    public IAction<Float> setPowerAction() {return setPowerAction;}
    public IAction<Boolean> setLockAction() {return setLockAction;}
    public IAction<Object> setMotorPowerAction() {return setMotorPowerAction;}

    class SetPowerAction implements IAction<Float> {

        @Override
        public void init(RobotController robot, Float data) {

        }

        @Override
        public void start(RobotController robot, Float data) {

        }

        @Override
        public void loop(RobotController robot, Float data) {
            if (!lock)
            {
                power = data;
            }
        }
    }

    class SetLockAction implements IAction<Boolean> {

        Toggle lockToggle;

        @Override
        public void init(RobotController robot, Boolean data) {
            lockToggle = new Toggle();
        }

        @Override
        public void start(RobotController robot, Boolean data) {

        }

        @Override
        public void loop(RobotController robot, Boolean data) {
            lockToggle.loop(data);

            lock = lockToggle.toggle();
        }

        public SetLockAction() {
            lockToggle = new Toggle();
        }
    }

    class SetMotorPowerAction implements IAction<Object> {

        @Override
        public void init(RobotController robot, Object data) {

        }
        @Override
        public void start(RobotController robot, Object data) {

        }

        @Override
        public void loop(RobotController robot, Object data) {
            motor.setPower(power);
        }
    }

    final IAction<Telemetry> telemetryAction = WithTelemetry.fromLambda(() -> "Flywheel Controller", telemetry -> {
        telemetry.addData("Power", power);
        telemetry.addData("Lock", lock);
    });

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction;
    }

    public FlywheelController(IMotor _motor) {
        motor = _motor;
        power = 0f;
        lock = false;

        setPowerAction = new SetPowerAction();
        setLockAction = new SetLockAction();
        setMotorPowerAction = new SetMotorPowerAction();
    }
}