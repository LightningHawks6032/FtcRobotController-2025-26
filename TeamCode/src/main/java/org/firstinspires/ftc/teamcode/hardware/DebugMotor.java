package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DebugMotor implements IMotor {
    float currentPower;
    Telemetry telemetry;
    String id;
    MotorSpec spec;
    public DebugMotor(String _id, Telemetry _telemetry, MotorSpec _spec) {
        id = _id;
        telemetry = _telemetry;
        spec = _spec;


    }

    @Override
    public void setPower(float power) {
        currentPower = power;
        telemetry.addData(id + " Power", currentPower);
    }

    @Override
    public float getPower() {
        return currentPower;
    }

    @Override
    public void setTorque(float torque_KgCm, float currentVelocity_RPM) {
        currentPower = torque_KgCm / spec.stallTorque + currentVelocity_RPM / spec.noLoadSpeed;
        telemetry.addData(id + " Torque", currentPower);
    }

    @Override
    public int getPosition() {
        return 0;
    }

    @Override
    public void setVelocity(float velocity_ticksPerSecond) {
        setPower(velocity_ticksPerSecond * 60 / (spec.noLoadSpeed * spec.encoderResolution));
    }

    @Override
    public float getVelocity() {
        return getPower() * spec.noLoadSpeed * spec.encoderResolution / 60;
    }

}
