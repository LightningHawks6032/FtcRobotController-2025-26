package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.components.RobotController;
import org.firstinspires.ftc.teamcode.components.action.ActionGroup;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.util.Vec2;

public  class InputResponseManager {
    GamepadWrapper gamepad;
    RobotController robot;

    public IAction<Vec2> leftStickAction;
    public IAction<Vec2> rightStickAction;
    public IAction<Vec2> dPadAction;
    public IAction<Boolean> AAction, BAction, XAction, YAction;
    public IAction<Boolean> leftBumperAction, rightBumperAction;
    public IAction<Float> leftTriggerAction, rightTriggerAction;

    public IAction<Object> loops;


    void onLeftTrigger() {
        leftTriggerAction.loop(robot, gamepad.leftTrigger());
    }
    void onRightTrigger() {
        rightTriggerAction.loop(robot, gamepad.rightTrigger());
    }

    void onLeftBumper() {
        leftBumperAction.loop(robot, gamepad.leftBumper());
    }
    void onRightBumper() {
        rightBumperAction.loop(robot, gamepad.rightBumper());
    }

    void onLeftStick() {
        leftStickAction.loop(robot, gamepad.leftStick());
    }
    void onRightStick() {
        rightStickAction.loop(robot, gamepad.rightStick());
    }

    void onA() {
        AAction.loop(robot, gamepad.pressedA());
    }
    void onX() {
        XAction.loop(robot, gamepad.pressedX());
    }
    void onY() {
        YAction.loop(robot, gamepad.pressedY());
    }
    void onB() {
        BAction.loop(robot, gamepad.pressedB());
    }

    void onDPad() {
        dPadAction.loop(robot, gamepad.DPad());
    }

    public void loop() {
        loops.loop(robot, 0);
        onLeftStick();
        onRightStick();
        onA();
        onB();
        onX();
        onY();
        onDPad();
        onRightBumper();
        onLeftBumper();
        onRightTrigger();
        onLeftTrigger();
    }
    
    public static class Builder {

        final InputResponseManager inputResponseManager;
        public Builder(GamepadWrapper _gamepad, RobotController _robot) {
            inputResponseManager = new InputResponseManager();
            inputResponseManager.gamepad = _gamepad;
            inputResponseManager.robot = _robot;
            inputResponseManager.leftStickAction = new EmptyAction<>();
            inputResponseManager.rightStickAction = new EmptyAction<>();
            inputResponseManager.AAction = new EmptyAction<>();
            inputResponseManager.BAction = new EmptyAction<>();
            inputResponseManager.XAction = new EmptyAction<>();
            inputResponseManager.YAction = new EmptyAction<>();
            inputResponseManager.dPadAction = new EmptyAction<>();
            inputResponseManager.loops = new EmptyAction<>();
            inputResponseManager.leftBumperAction = new EmptyAction<>();
            inputResponseManager.rightBumperAction = new EmptyAction<>();
            inputResponseManager.leftTriggerAction = new EmptyAction<>();
            inputResponseManager.rightTriggerAction = new EmptyAction<>();
        }

        @SafeVarargs
        public final Builder rightTriggerAction(IAction<Float>... _rightTriggerAction) {
            inputResponseManager.rightTriggerAction = new ActionGroup<>(_rightTriggerAction);
            return this;
        }

        @SafeVarargs
        public final Builder leftTriggerAction(IAction<Float>... _leftTriggerAction) {
            inputResponseManager.leftTriggerAction = new ActionGroup<>(_leftTriggerAction);
            return this;
        }

        @SafeVarargs
        public final Builder rightBumperAction(IAction<Boolean>... _rightBumperAction) {
            inputResponseManager.rightBumperAction = new ActionGroup<>(_rightBumperAction);
            return this;
        }

        @SafeVarargs
        public final Builder leftBumperAction(IAction<Boolean>... _leftBumperAction) {
            inputResponseManager.leftBumperAction= new ActionGroup<>(_leftBumperAction);
            return this;
        }

        @SafeVarargs
        public final Builder leftStickAction(IAction<Vec2>... _leftStickAction) {
            inputResponseManager.leftStickAction = new ActionGroup<>(_leftStickAction);
            return this;
        }
        @SafeVarargs
        public final Builder rightStickAction(IAction<Vec2>... _rightStickAction) {
            inputResponseManager.rightStickAction = new ActionGroup<>(_rightStickAction);
            return this;
        }

        @SafeVarargs
        public final Builder AAction(IAction<Boolean>... _AAction) {
            inputResponseManager.AAction = new ActionGroup<>(_AAction);
            return this;
        }
        @SafeVarargs
        public final Builder BAction(IAction<Boolean>... _BAction) {
            inputResponseManager.BAction = new ActionGroup<>(_BAction);
            return this;
        }
        @SafeVarargs
        public final Builder XAction(IAction<Boolean>... _XAction) {
            inputResponseManager.XAction = new ActionGroup<>(_XAction);
            return this;
        }
        @SafeVarargs
        public final Builder YAction(IAction<Boolean>... _YAction) {
            inputResponseManager.YAction = new ActionGroup<>(_YAction);
            return this;
        }
        @SafeVarargs
        public final Builder DPadAction(IAction<Vec2>... _dPadAction) {
            inputResponseManager.dPadAction = new ActionGroup<>(_dPadAction);
            return this;
        }

        @SafeVarargs
        public final Builder loops(IAction<Object>... _loops) {
            inputResponseManager.loops = new ActionGroup<>(_loops);
            return this;
        }

        public final InputResponseManager build() {
            return inputResponseManager;
        }
    }
}
