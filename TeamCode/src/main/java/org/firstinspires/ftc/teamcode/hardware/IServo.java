package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.util.WithTelemetry;

public interface IServo extends WithTelemetry.IWithTelemetry{

    float getPosition();
    void setPosition(float position);

    float getVelocity();
    void setVelocity(float velocity);

    IMotor.Direction getDirection();
    void setDirection(IMotor.Direction dir);
}
