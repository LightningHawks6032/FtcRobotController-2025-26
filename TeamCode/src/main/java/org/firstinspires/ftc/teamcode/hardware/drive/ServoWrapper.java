package org.firstinspires.ftc.teamcode.hardware.drive;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.IServo;

/// Assumes servo is position-based
public class ServoWrapper implements IServo {
    float position = 0;
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
        position = _position;
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
    }
}
