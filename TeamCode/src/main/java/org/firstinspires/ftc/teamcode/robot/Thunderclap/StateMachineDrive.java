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
        LOOK_AT_USER_DIRECTION,
        LOOK_AT_APRIL_TAG_FIXED
    }
    final DirectDrive directDrive;
    final ActionStateMachine<State, Object> stateMachine;
    public IAction<Object> stateMachineAction() {return stateMachine;}
    final IControlLoop controlLoop;
    final Supplier<Float> currentCameraYaw;
    final Supplier<Float> currentYaw;
    float lastHeading;

    boolean lookAtAprilTag;
    IAction<Boolean> lookAtAprilTagAction;
    public IAction<Boolean> lookAtAprilTagAction() {return lookAtAprilTagAction;}

    float rotationPower;
    IAction<Float> controlLoopAction;
    public IAction<Float> controlLoopAction() {return controlLoopAction;}

    Toggle lookAtAprilTagFixedToggle;
    float lookAtAprilTagFixedPower;
    final Vec2 aprilTagPosition = new Vec2(50, 0);//Vec2.zero();
    float aprilTagFixedAngle;
    float aprilTagFixedControlLoopPower;
    Supplier<Vec2Rot> robotPos;
    Supplier<Float> aprilTagDistance;
    IControlLoop aprilTagFixedControlLoop;
    void findAprilTagPos() {

        if (false && isReading.get()) {


            /*float d = aprilTagDistance.get();
            float x = (float) Math.sqrt(17.5 * 17.5 + d * d - 17.5f * d * Math.cos(currentCameraYaw.get()));
            float theta = (float) Math.asin(d / x * Math.sin(currentCameraYaw.get()));
            aprilTagPosition = robotPos.get().asVec2().add(new Vec2(
                    (float) Math.cos(theta),
                    (float) Math.sin(theta)
            ).scale(x));*/
            /*aprilTagPosition = robotPos.get().asVec2().add(
                    new Vec2(
                            (float)Math.cos(currentCameraYaw.get()),
                            (float)Math.sin(currentCameraYaw.get())
                    ).scale(aprilTagDistance.get())
            );*/
        }
        Vec2 diff = aprilTagPosition.sub(robotPos.get().asVec2());
        aprilTagFixedAngle = (float)Math.atan2(diff.x, diff.y);
    }
    IAction<Boolean> lookAtAprilTagActionFixed;
    public IAction<Boolean> lookAtAprilTagActionFixed() {return lookAtAprilTagActionFixed;}

    boolean lookAtUserDirection;
    /// in radians;
    float userDirection;
    float desiredUserDirectionPower;
    Toggle userDirectionToggle;
    final IControlLoop userDirectionControlLoop;
    final IAction<Vec2> lookAtUserDirectionAction;
    public IAction<Vec2> lookAtUserDirectionAction() {return lookAtUserDirectionAction;}

    Supplier<Boolean> isReading;

    public StateMachineDrive(DirectDrive _directDrive,
                             @NonNull IControlLoopBuildOpt<? extends IControlLoop> _controlLoop,
                             Supplier<Float> _getCurrentYaw,
                             Supplier<Float> _getCurrentRobotYaw,
                             Supplier<Boolean> _isReading,
                             Supplier<Vec2Rot> _robotPos,
                             Supplier<Float> _cameraDistance) {
        directDrive = _directDrive;
        controlLoop = _controlLoop.build();
        userDirectionControlLoop = _controlLoop.build();
        aprilTagFixedControlLoop = _controlLoop.build();
        currentCameraYaw = _getCurrentYaw;
        currentYaw = _getCurrentRobotYaw;
        robotPos = _robotPos;
        aprilTagDistance = _cameraDistance;
        isReading = _isReading;
        lastHeading = 0f;

        userDirectionToggle = new Toggle();

        lookAtAprilTagFixedToggle = new Toggle();
        lookAtAprilTagFixedPower = 0f;

        lookAtAprilTag = false;
        lookAtAprilTagAction = IAction.From.loop((r, b) -> lookAtAprilTag = b);

        lookAtAprilTagActionFixed = IAction.From.loop((r, b) -> lookAtAprilTagFixedToggle.loop(b));

        lookAtUserDirection = false;
        desiredUserDirectionPower = 0f;
        userDirection = 0f;
        lookAtUserDirectionAction = IAction.From.loop((r, v) -> {
            lookAtUserDirection = true;
            if (v.y > 0) {
                userDirection = 0f;
            }
            else if (v.y < 0) {
                userDirection = (float)Math.PI * (1 - 1/2f);
            }
            else if (v.x > 0) {
                userDirection = (float)Math.PI * (1 - 1/12f);
            }
            else if (v.x < 0) {
                userDirection = -(float)Math.PI * (1 - 1/4f);
            }
            else {
                lookAtUserDirection = false;
            }
        });

        rotationPower = 0f;
        controlLoopAction = IAction.From.loop((r, f) -> {
            rotationPower = controlLoop.loop(Util.computeShortestSignedAngle(Util.normAngle2Pi(currentYaw.get()), Util.normAngle2Pi(lastHeading + currentCameraYaw.get())), 0f, f);
            desiredUserDirectionPower = userDirectionControlLoop.loop(Util.computeShortestSignedAngle(Util.normAngle2Pi(currentYaw.get()), Util.normAngle2Pi(userDirection + (float)Math.PI)), 0f, f);
            findAprilTagPos();
            if (isReading.get()) {
                lastHeading = currentYaw.get();
            }
            aprilTagFixedControlLoopPower = aprilTagFixedControlLoop.loop(
                    Util.computeShortestSignedAngle(Util.normAngle2Pi(currentYaw.get()), Util.normAngle2Pi(aprilTagFixedAngle)), 0f, f
            );
        });

        IAction<Object> directDriveAction = directDrive.splitAction();
        IAction<Object> lookAction = IAction.From.loop((r, o) -> {
            directDrive.directDriveAction().loop(r, new Vec2Rot(
                    directDrive.getLastStickInput().asVec2(),
                    Util.clamp(rotationPower, -0.6f, 0.6f)
            ));
        });
        IAction<Object> userDirectionAction = IAction.From.loop((r, o) -> {
            directDrive.directDriveAction().loop(r, new Vec2Rot(
                    directDrive.getLastStickInput().asVec2(),
                    Util.clamp(desiredUserDirectionPower, -0.6f, 0.6f)
            ));
        });

        IAction<Object> lookAprilTagFixed = IAction.From.loop((r, o) -> {
            directDrive.directDriveAction().loop(r, new Vec2Rot(
                    directDrive.getLastStickInput().asVec2(),
                    Util.clamp(aprilTagFixedControlLoopPower, -0.6f, 0.6f)
            ));
        });

        stateMachine = new ActionStateMachine<>(state -> {
            switch (state) {
                case DIRECT:
                    return directDriveAction;
                case LOOK_AT_APRIL_TAG:
                    return lookAction;
                case LOOK_AT_USER_DIRECTION:
                    return userDirectionAction;
                case LOOK_AT_APRIL_TAG_FIXED:
                    return lookAprilTagFixed;
            }
            return directDriveAction;
        });

        stateMachine.addNode(State.DIRECT);

        stateMachine.addNode(State.LOOK_AT_APRIL_TAG, controlLoop::reset, () -> {}, () -> {});

        stateMachine.addNode(State.LOOK_AT_USER_DIRECTION, userDirectionControlLoop::reset, () -> {}, () -> {});

        stateMachine.addNode(State.LOOK_AT_APRIL_TAG_FIXED, aprilTagFixedControlLoop::reset, ()->{}, ()->{});

        stateMachine.addSwitch(State.DIRECT,
                State.LOOK_AT_APRIL_TAG,
                () -> lookAtAprilTag
        );

        stateMachine.addSwitch(State.DIRECT, State.LOOK_AT_USER_DIRECTION, () -> {
            //userDirectionToggle.loop(lookAtUserDirection);
            return lookAtUserDirection;//userDirectionToggle.toggle();
        });

        stateMachine.addSwitch(State.DIRECT, State.LOOK_AT_APRIL_TAG_FIXED, lookAtAprilTagFixedToggle::toggle);

        stateMachine.setCurrentNode(State.DIRECT);


    }

}
