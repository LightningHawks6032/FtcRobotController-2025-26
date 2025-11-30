package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.ActionGroup;
import org.firstinspires.ftc.teamcode.components.action.ActionStateMachine;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.control.DirectControlLoop;
import org.firstinspires.ftc.teamcode.control.IControlLoop;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.util.LazyInit;
import org.firstinspires.ftc.teamcode.util.Toggle;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

public class OuttakeWheelController implements WithTelemetry.IWithTelemetry {
    final DcMotorWrapper motor;
    float targetSpeed;
    final LazyInit<IAction<Float>> setMotorSpeedAction;
    /// Sets motor speed as proportion of max rpm
    /// Will eventually need to be replaced with an absolute rpm
    public IAction<Float> setMotorSpeedAction() {return setMotorSpeedAction.get();}

    Toggle motorSpeedToggle;
    public LazyInit<IAction<Boolean>> setMotorSpeedButton;
    /// Sets the motor speed to max with a button toggle
    /// This is **instead** of `setMotorSpeedAction` as they both
    /// override each other in setting motor power each frame
    public IAction<Boolean> setMotorSpeedButtonAction() {return setMotorSpeedButton.get();}

    final IControlLoop controlLoop;
    float controlLoopOutput;
    final LazyInit<IAction<Float>> controlLoopAction;
    /// Time loop action
    public IAction<Float> controlLoopAction() {return controlLoopAction.get();}


    final Toggle lockToggle;
    final LazyInit<IAction<Boolean>> lockToggleAction;
    /// Toggles motor speed lock
    public IAction<Boolean> lockToggleAction() {return lockToggleAction.get();}

    final LazyInit<IAction<Object>> setMotorPowerAction;
    /// Sets the physical motor power
    public IAction<Object> setMotorPowerAction() {return setMotorPowerAction.get();}

    static class DistanceSpeedComputer {
        Supplier<AprilTagDetection> lastReading;

        final float coefficient = 0.00344f;
        final float offset = 0.299f;
        final float fallback = 0.67f;

        float getPower(float distance) {
            return offset + coefficient * distance;
        }

        public float getDesiredPower() {
            AprilTagDetection last = lastReading.get();

            return last == null ? fallback : getPower((float)last.ftcPose.range);
        }

        public DistanceSpeedComputer(Supplier<AprilTagDetection> _lastReading) {
            lastReading = _lastReading;
        }
    }

    enum StateMachineControlState {
        /// Spin backwards slowly
        IDLE,
        /// By controller power (`OuttakeWheelController.setMotorPowerAction`)
        CONTROLLER,
        /// Adjust by distance measured from camera
        DISTANCE
    }
    class StateMachineControl {

        final ActionStateMachine<StateMachineControlState, Object> stateMachine;
        final Toggle controlSpeedToggle, isIdleToggle;
        final DistanceSpeedComputer speedComputer;

        public final IAction<Boolean> controlSpeedToggleAction, idleToggleAction;
        public final IAction<Object> stateMachineAction;

        final float IDLE_SPIN_POWER = -0.3f;

        public StateMachineControl(DistanceSpeedComputer _speedComputer) {
            speedComputer = _speedComputer;

            IAction<Object> idleSpinAction = IAction.From.loop((r, o) -> motor.setPower(IDLE_SPIN_POWER));
            IAction<Object> distanceAction = new ActionGroup<>(
                    IAction.From.loop((r, o) -> targetSpeed = speedComputer.getDesiredPower()),
                    setMotorPowerAction.get()
            );

            controlSpeedToggle = new Toggle(true);
            isIdleToggle = new Toggle(true);

            stateMachine = new ActionStateMachine<>((state) -> {
                switch (state) {
                    case IDLE:
                        return idleSpinAction;
                    case CONTROLLER:
                        return setMotorPowerAction.get();
                    case DISTANCE:
                        return distanceAction;

                }
                return null;
            });

            stateMachine.addNode(StateMachineControlState.IDLE);
            stateMachine.addNode(StateMachineControlState.CONTROLLER);
            stateMachine.addNode(StateMachineControlState.DISTANCE);

            /// ESCAPE IDLE IF IDLE TOGGLE IS OFF
            stateMachine.addArrow(
                    StateMachineControlState.IDLE,
                    StateMachineControlState.CONTROLLER,
                    () -> !isIdleToggle.toggle() && controlSpeedToggle.toggle()
            );
            stateMachine.addArrow(
                    StateMachineControlState.IDLE,
                    StateMachineControlState.DISTANCE,
                    () -> !isIdleToggle.toggle() && !controlSpeedToggle.toggle()
            );
            /// GO TO IDLE IF IDLE TOGGLE IS ON
            stateMachine.addArrow(
                    StateMachineControlState.CONTROLLER,
                    StateMachineControlState.IDLE,
                    isIdleToggle::toggle
            );
            stateMachine.addArrow(
                    StateMachineControlState.DISTANCE,
                    StateMachineControlState.IDLE,
                    isIdleToggle::toggle
            );
            /// SWITCH BETWEEN CONTROLLER AND DISTANCE VIA TOGGLE
            stateMachine.addArrow(
                    StateMachineControlState.CONTROLLER,
                    StateMachineControlState.DISTANCE,
                    () -> !controlSpeedToggle.toggle()
            );
            stateMachine.addArrow(
                    StateMachineControlState.DISTANCE,
                    StateMachineControlState.CONTROLLER,
                    controlSpeedToggle::toggle
            );

            stateMachine.setCurrentNode(StateMachineControlState.IDLE);


            controlSpeedToggleAction = IAction.From.loop((r, b) -> controlSpeedToggle.loop(b));
            idleToggleAction = IAction.From.loop((r, b) -> isIdleToggle.loop(b));

            stateMachineAction = stateMachine;
        }
    }

    LazyInit<StateMachineControl> stateMachineControl;
    public IAction<Object> stateMachineAction() {return stateMachineControl.get().stateMachineAction;}
    public IAction<Boolean> stateMachineControlSpeedToggleAction() {return stateMachineControl.get().controlSpeedToggleAction;}
    public IAction<Boolean> stateMachineIdleToggleAction() {return stateMachineControl.get().idleToggleAction;}
    public WithTelemetry.IWithTelemetry stateMachineTelemetry() {return stateMachineControl.get().stateMachine;}


    public OuttakeWheelController(DcMotorWrapper _motor, IControlLoop _controlLoop, Supplier<AprilTagDetection> _lastDetection) {
        motor = _motor;
        targetSpeed = 0f;
        controlLoop = _controlLoop;

        lockToggle = new Toggle(false);
        motorSpeedToggle = new Toggle(false);

        setMotorSpeedAction = new LazyInit<>(() ->
            IAction.From.loop((robot, f) ->
                {if (!lockToggle.toggle()) targetSpeed = f;}
            )
        );

        setMotorSpeedButton = new LazyInit<>(() -> IAction.From.loop(
                (r, b) -> {
                    motorSpeedToggle.loop(b);
                    targetSpeed = motorSpeedToggle.toggle() ? 1:0;
                }
        ));

        controlLoopAction = new LazyInit<>(() ->
            IAction.From.loop((r, f) ->
                    controlLoopOutput = controlLoop.loop(
                            motor.getSpec().proportionOfNoLoad(motor.getVelocityRPM()),
                            targetSpeed,
                            f
                    )
        ));

        lockToggleAction = new LazyInit<>(() ->
                IAction.From.loop((r, b) -> lockToggle.loop(b)));

        setMotorPowerAction = new LazyInit<>(() ->
            IAction.From.loop((r, o) -> motor.setPower(controlLoopOutput))
        );

        stateMachineControl = new LazyInit<>(() -> new StateMachineControl(
                new DistanceSpeedComputer(_lastDetection)
        ));

        telemetryAction = new LazyInit<>(() ->
            WithTelemetry.fromLambda(() -> "Outtake Wheel Controller", (telemetry) -> {
                telemetry.addData("Power", controlLoopOutput);
                telemetry.addData("Velocity (rpm)", motor.getSpec().proportionOfNoLoad(motor.getVelocityRPM()));
                telemetry.addData("Target Velocity (rpm)", targetSpeed);
                telemetry.addData("Lock", lockToggle.toggle());
            })
        );
    }
    public OuttakeWheelController(DcMotorWrapper _motor, Supplier<AprilTagDetection> _lastDetection) {
        this(_motor, new DirectControlLoop.Controller(), _lastDetection);
    }

    final LazyInit<IAction<Telemetry>> telemetryAction;
    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction.get();
    }
}
