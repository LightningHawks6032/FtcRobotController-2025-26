package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.control.IControlLoop;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.IServo;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;
import org.firstinspires.ftc.teamcode.util.Toggle;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

/// `setPowerAction` - Sets the internal motor power<br>
/// `setLockAction` - Locks the internal motor power<br>
/// `setMotorPowerAction` - Sets the physical motor power<br>
/// `setHoodPowerAction` - Sets power on hood servo
public class FlywheelController implements WithTelemetry.IWithTelemetry {
    IMotor motor;
    IServo servo;
    float power;
    boolean lock;
    float lockRPM;
    IControlLoop controlLoop;

    final SetPowerAction setPowerAction;
    final SetLockAction setLockAction;
    final SetMotorPowerAction setMotorPowerAction;
    final SetHoodPowerAction setHoodPowerAction;

    public IAction<Float> setPowerAction() {return setPowerAction;}
    public IAction<Boolean> setLockAction() {return setLockAction;}
    public IAction<Object> setMotorPowerAction() {return setMotorPowerAction;}
    public IAction <Float> setHoodPowerAction() {return setHoodPowerAction;}

    class SetPowerAction implements IAction<Float> {

        @Override
        public void init(IRobot robot, Float data) {

        }

        @Override
        public void start(IRobot robot, Float data) {

        }

        @Override
        public void loop(IRobot robot, Float data) {
            if (!lock)
            {
                power = data;
            }
        }
    }

    class SetLockAction implements IAction<Boolean> {

        Toggle lockToggle;

        @Override
        public void init(IRobot robot, Boolean data) {
            lockToggle = new Toggle();
        }

        @Override
        public void start(IRobot robot, Boolean data) {

        }

        @Override
        public void loop(IRobot robot, Boolean data) {
            lockToggle.loop(data);

            lock = lockToggle.toggle();
        }

        public SetLockAction() {
            lockToggle = new Toggle();
        }
    }

    class SetMotorPowerAction implements IAction<Object> {

        @Override
        public void init(IRobot robot, Object data) {

        }
        @Override
        public void start(IRobot robot, Object data) {

        }

        @Override
        public void loop(IRobot robot, Object data) {
            motor.setPower(power);
        }
    }

    class SetHoodPowerAction implements IAction<Float> {

        @Override
        public void init(IRobot robot, Float data) {

        }

        @Override
        public void start(IRobot robot, Float data) {

        }

        @Override
        public void loop(IRobot robot, Float data) {
            servo.setVelocity(data);
        }
    }

    final IAction<Telemetry> telemetryAction = WithTelemetry.fromLambda(() -> "Flywheel Controller", telemetry -> {
        telemetry.addData("Power", power);
        telemetry.addData("Lock", lock);
        telemetry.addData("Hood Position", servo.getPosition());
    });

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction;
    }

    public FlywheelController(IMotor _motor, IServo _servo) {
        motor = _motor;
        servo = _servo;

        power = 0f;
        lock = false;
        lockRPM = 0f;

        setPowerAction = new SetPowerAction();
        setLockAction = new SetLockAction();
        setMotorPowerAction = new SetMotorPowerAction();
        setHoodPowerAction = new SetHoodPowerAction();
    }
}