package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.util.Util;

/// Assumes servo is position-based
public class ServoWrapper implements IServo {
    float position;
    boolean moving = false;
    IMotor.Direction direction;
    int directionCoeff;
    final float speed;
    Servo servo;

    @Override
    public float getPosition() {
        return position;
    }
    @Override
    public void setPosition(float _position) {
        position = Util.clamp(_position, 0, 1);
        servo.setPosition(position);
    }
    @Override
    public void setVelocity(float velocity) {
        if (velocity == 0) {
            moving = false;
        }
        else {
            moving = true;
            setPosition(position + velocity * directionCoeff * speed);
        }
    }

    @Override
    public float getVelocity() {
        return moving ? speed * directionCoeff : 0;
    }

    @Override
    public void setDirection(IMotor.Direction dir) {
        direction = dir;
        directionCoeff = (direction == IMotor.Direction.FORWARD ? 1 : -1);
    }

    @Override
    public IMotor.Direction getDirection() {
        return direction;
    }

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return null;
    }

    public ServoWrapper(Servo _servo) {
        servo = _servo;
        direction = IMotor.Direction.FORWARD;
        directionCoeff = 1;
        speed = 0.05f;
        position = (float)_servo.getPosition();
    }

    public ServoWrapper(Servo _servo, float _speed) {
        servo = _servo;
        direction = IMotor.Direction.FORWARD;
        directionCoeff = 1;
        speed = _speed;
        position = (float)_servo.getPosition();
    }
}
