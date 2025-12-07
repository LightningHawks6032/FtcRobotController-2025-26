package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Vec2;

public class GamepadWrapper {
    final Gamepad gamepad;

    public float leftStickY() {return gamepad.left_stick_y;}
    public float leftStickX() {return gamepad.left_stick_x;}
    public float rightStickX() {return gamepad.right_stick_x;}
    public float rightStickY() {return gamepad.right_stick_y;}
    public Vec2 leftStick() {return new Vec2(gamepad.left_stick_x, gamepad.left_stick_y);}
    public Vec2 rightStick() {return new Vec2(gamepad.right_stick_x, gamepad.right_stick_y);}
    public boolean pressedA() {return gamepad.a;}
    public boolean pressedB() {return gamepad.b;}
    public boolean pressedX() {return gamepad.x;}
    public boolean pressedY() {return gamepad.y;}
    public int bumper() {return (gamepad.right_bumper?1:0) - (gamepad.left_bumper?1:0);}
    public float trigger() {return gamepad.right_trigger - gamepad.left_trigger;}
    public float leftTrigger() {return gamepad.left_trigger;}
    public float rightTrigger() {return gamepad.right_trigger;}
    public int horizontalDPad() {return (gamepad.dpad_right?1:0) - (gamepad.dpad_left?1:0);}
    public int verticalDPad() {return (gamepad.dpad_up?1:0) - (gamepad.dpad_down?1:0);}
    public boolean leftBumper() {return gamepad.left_bumper;}
    public boolean rightBumper() {return gamepad.right_bumper;}
    public Vec2 DPad() {return new Vec2(horizontalDPad(), verticalDPad());}
    public GamepadWrapper(Gamepad _gamepad) {
        gamepad = _gamepad;
    }

    public void rumble() {
        gamepad.rumble(100);
    }

}
