package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.DirectDrive;
import org.firstinspires.ftc.teamcode.components.action.ActionStateMachine;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.control.IControlLoop;
import org.firstinspires.ftc.teamcode.control.IControlLoopBuildOpt;
import org.firstinspires.ftc.teamcode.util.Toggle;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

import java.util.function.Supplier;

public class StateMachineDrive implements WithTelemetry.IWithTelemetry {

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return WithTelemetry.fromLambda(() -> "State Machine Drive", telemetry -> {
            telemetry.addData("User direction angle (rad)", userDirection);
            telemetry.addData("IMU read (rad)", currentYaw.get());
            telemetry.addData("User direction control loop", desiredUserDirectionPower);
            telemetry.addData("State", stateMachine.getCurrentNode());
            telemetry.addData("User Direction PID Inputs",  Util.normAngle2Pi(currentYaw.get()) + ", " + Util.normAngle2Pi(userDirection));
        });
    }

    enum State {
        DIRECT,
        LOOK_AT_APRIL_TAG,
        LOOK_AT_USER_DIRECTION
    }
    final DirectDrive directDrive;
    final ActionStateMachine<State, Object> stateMachine;
    public IAction<Object> stateMachineAction() {return stateMachine;}
    final IControlLoop controlLoop;
    final Supplier<Float> currentCameraYaw;
    final Supplier<Float> currentYaw;

    boolean lookAtAprilTag;
    IAction<Boolean> lookAtAprilTagAction;
    public IAction<Boolean> lookAtAprilTagAction() {return lookAtAprilTagAction;}

    float rotationPower;
    IAction<Float> controlLoopAction;
    public IAction<Float> controlLoopAction() {return controlLoopAction;}

    static final float DEG_TO_RAD = (float)Math.PI / 180;

    boolean lookAtUserDirection;
    /// in radians;
    float userDirection;
    float desiredUserDirectionPower;
    Toggle userDirectionToggle;
    final IControlLoop userDirectionControlLoop;
    final IAction<Vec2> lookAtUserDirectionAction;
    public IAction<Vec2> lookAtUserDirectionAction() {return lookAtUserDirectionAction;}
    public StateMachineDrive(DirectDrive _directDrive,
                             @NonNull IControlLoopBuildOpt<? extends IControlLoop> _controlLoop,
                             Supplier<Float> _getCurrentYaw,
                             Supplier<Float> _getCurrentRobotYaw) {
        directDrive = _directDrive;
        controlLoop = _controlLoop.build();
        userDirectionControlLoop = _controlLoop.build();
        currentCameraYaw = _getCurrentYaw;
        currentYaw = _getCurrentRobotYaw;

        userDirectionToggle = new Toggle();

        lookAtAprilTag = false;
        lookAtAprilTagAction = IAction.From.loop((r, b) -> lookAtAprilTag = b);

        lookAtUserDirection = false;
        desiredUserDirectionPower = 0f;
        userDirection = 0f;
        lookAtUserDirectionAction = IAction.From.loop((r, v) -> {
            lookAtUserDirection = true;
            if (v.y > 0) {
                userDirection = 0f;
            }
            else if (v.y < 0) {
                userDirection = (float)Math.PI/2;
            }
            else if (v.x > 0) {
                userDirection = (float)Math.PI/4;
            }
            else if (v.x < 0) {
                userDirection = -(float)Math.PI/4;
            }
            else {
                lookAtUserDirection = false;
            }
        });

        rotationPower = 0f;
        controlLoopAction = IAction.From.loop((r, f) -> {
            rotationPower = controlLoop.loop(Util.normAngle2Pi(currentYaw.get()), Util.normAngle2Pi(currentYaw.get() + currentCameraYaw.get()), f);
            desiredUserDirectionPower = userDirectionControlLoop.loop(Util.normAngle2Pi(currentYaw.get()), Util.normAngle2Pi(userDirection), f);
        });

        IAction<Object> directDriveAction = directDrive.splitAction();
        IAction<Object> lookAction = IAction.From.loop((r, o) -> {
            directDrive.directDriveAction().loop(r, new Vec2Rot(0, 0, rotationPower));
        });

        IAction<Object> userDirectionAction = IAction.From.loop((r, o) -> {
            directDrive.directDriveAction().loop(r, new Vec2Rot(0, 0, Util.clamp(desiredUserDirectionPower, -0.6f, 0.6f)));
        });


        stateMachine = new ActionStateMachine<>(state -> {
            switch (state) {
                case DIRECT:
                    return directDriveAction;
                case LOOK_AT_APRIL_TAG:
                    return lookAction;
                case LOOK_AT_USER_DIRECTION:
                    return userDirectionAction;
            }
            return directDriveAction;
        });

        stateMachine.addNode(State.DIRECT);

        stateMachine.addNode(State.LOOK_AT_APRIL_TAG, controlLoop::reset, () -> {}, () -> {});

        stateMachine.addNode(State.LOOK_AT_USER_DIRECTION, userDirectionControlLoop::reset, () -> {}, () -> {});

        stateMachine.addSwitch(State.DIRECT,
                State.LOOK_AT_APRIL_TAG,
                () -> lookAtAprilTag
        );

        stateMachine.addSwitch(State.DIRECT, State.LOOK_AT_USER_DIRECTION, () -> {
            userDirectionToggle.loop(lookAtUserDirection);
            return userDirectionToggle.toggle();
        });

        stateMachine.setCurrentNode(State.DIRECT);


    }

}
