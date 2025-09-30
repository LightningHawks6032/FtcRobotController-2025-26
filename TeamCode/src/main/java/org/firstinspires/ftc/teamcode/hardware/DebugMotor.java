package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DebugMotor implements IMotor {
    float currentPower;
    final Telemetry telemetry;
    final String id;
    final MotorSpec spec;
    Direction dir;
    public DebugMotor(String _id, Telemetry _telemetry, MotorSpec _spec) {
        id = _id;
        telemetry = _telemetry;
        spec = _spec;
        dir = Direction.FORWARD;
    }

    public static class BuildOpt implements IMotorBuildOpt<DebugMotor>{
        final String name;
        final MotorSpec spec;
        final Telemetry telemetry;

        public BuildOpt(String _name, Telemetry _telemetry, MotorSpec _spec) {
            name = _name;
            telemetry = _telemetry;
            spec = _spec;
        }

        @Override
        @NonNull
        public DebugMotor fromMap(@NonNull HardwareMap.DeviceMapping<DcMotor> _map) {
            return new DebugMotor(name, telemetry, spec);
        }
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

    @Override
    public void setDirection(Direction _dir) {
        dir = _dir;
    }

    @Override
    public Direction getDirection() {
        return dir;
    }

}
