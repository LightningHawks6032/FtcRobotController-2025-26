package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.IServo;
import org.firstinspires.ftc.teamcode.util.Toggle;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;


/// `setPowerAction` - Sets the internal motor power<br>
/// `setLockAction` - Locks the internal motor power<br>
/// `setMotorPowerAction` - Sets the physical motor power
public class IntakeMotorController implements WithTelemetry.IWithTelemetry {
    IMotor motor;
    float power;
    boolean lock;

    final IAction<Float> setPowerAction;
    final SetLockAction setLockAction;
    final IAction<Object> setMotorPowerAction;
    final IAction<Telemetry> telemetryAction;


    public IAction<Float> setPowerAction() {return setPowerAction;}
    public IAction<Boolean> setLockAction() {return setLockAction;}
    public IAction<Object> setMotorPowerAction() {return setMotorPowerAction;}

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction;
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

    public IntakeMotorController(IMotor _motor) {
        motor = _motor;

        setPowerAction = IAction.From.loop(
                (r, p) -> {
                    if (!lock) {power = p;}
                }
        );

        setLockAction = new SetLockAction();

        setMotorPowerAction = IAction.From.loop(
                (r, o) -> motor.setPower(power)
        );

        telemetryAction = WithTelemetry.fromLambda(() -> "Intake Controller", (telemetry) -> {
            telemetry.addData("power", power);
            telemetry.addData("lock", lock);
        });
    }

}
