package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

public class DcMotorWrapper implements IMotor {
    public class Encoder {
        public boolean runningToPosition = false;
        public void setPosition(int position) {
            if (!usingEncoder) {System.err.println("Tried to set position on non-encoder motor" + motor.getDeviceName());}
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runningToPosition = true;
        }
        public void resetEncoder() {

            if (!usingEncoder) {System.err.println("Tried to reset encoder on non-encoder motor " + motor.getDeviceName()); return;}
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void lock() {

            if (!usingEncoder) {System.err.println("Tried to lock non-encoder motor" + motor.getDeviceName());}
            setPosition(getPosition());
            setPower(1);
        }

        public void unlock() {

            if (!usingEncoder) {System.err.println("Tried to unlock non-encoder motor" + motor.getDeviceName());}
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setPower(0);
        }
        // Returns Ticks per Second
        public float getVelocity() {
            return (float)motor.getVelocity();
        }
    }

    public final Encoder encoder;
    final DcMotorEx motor;
    final MotorSpec spec;
    final boolean usingEncoder;


    public static class BuildOpt implements IMotorBuildOpt<DcMotorWrapper>{
        final String name;
        final boolean usingEncoder;
        final MotorSpec spec;

        public BuildOpt(String _name, boolean _usingEncoder, MotorSpec _spec) {
            name = _name;
            usingEncoder = _usingEncoder;
            spec = _spec;
        }

        @Override
        @NonNull
        public DcMotorWrapper fromMap(@NonNull HardwareMap.DeviceMapping<DcMotor> _map) {
            return new DcMotorWrapper(_map.get(name), usingEncoder, spec);
        }
    }

    public DcMotorWrapper(DcMotor _motor, boolean _usingEncoder, MotorSpec _spec) {
        motor = (DcMotorEx)_motor;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //TODO: Fix this lmao
        spec = _spec == null ? MotorSpec.GOBILDA_5000_0002_0001 : _spec;
        usingEncoder = _usingEncoder;
        encoder = new Encoder();


    }
    public void setDirection(IMotor.Direction dir) {
        motor.setDirection(IMotor.Direction.toMotorDir(dir));
    }

    @Override
    public Direction getDirection() {
        return IMotor.Direction.fromMotorDir(motor.getDirection());
    }

    @Override
    public void setPower(float power) {
        motor.setPower(power);
    }

    @Override
    public float getPower() {
        return (float)motor.getPower();
    }

    @Override
    public void setTorque(float torque_KgCm, float currentVelocity_RPM) {
        setPower(torque_KgCm / spec.stallTorque + currentVelocity_RPM / spec.noLoadSpeed);
    }

    @Override
    public int getPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void setVelocity(float velocity_ticksPerSecond) {
        setPower(velocity_ticksPerSecond * 60 / (spec.noLoadSpeed * spec.encoderResolution));
    }


    // Returns Ticks per Second
    @Override
    public float getVelocity() {
        if (usingEncoder) {
            return encoder.getVelocity();
        }
        return getPower() * spec.noLoadSpeed * spec.encoderResolution / 60;
    }

    public float getVelocityRPM() {
        return spec.ticksPerSecondToRPM(getVelocity());
    }

    public MotorSpec getSpec() {
        return spec;
    }


    IAction<Telemetry> telem = new WithTelemetry.Action<WithTelemetry.ITelemetry>(
            new WithTelemetry.ITelemetry() {
                @Override
                public String getName() {
                    return motor.getDeviceName();
                }

                @Override
                public void loop(Telemetry _telemetry) {
                    _telemetry.addData("Pos (ticks)", getPosition());
                    _telemetry.addData("Vel (ticks/s)", getVelocity());
                    _telemetry.addData("Power", motor.getPower());
                }
            }
    );
    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telem;
    }
}
