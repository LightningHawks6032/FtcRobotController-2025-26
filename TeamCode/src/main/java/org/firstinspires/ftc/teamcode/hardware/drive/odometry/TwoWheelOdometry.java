package org.firstinspires.ftc.teamcode.hardware.drive.odometry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.drive.IIMU;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.deadwheel.DeadwheelWrapper;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;
import org.jetbrains.annotations.Contract;

public class TwoWheelOdometry implements IOdometry {
    Vec2Rot pos, vel, acc;

    final WheelSpec wheelSpec;
    final Wheels wheels;
    IIMU imu;

    Vec2 prevPos;
    float prevHeading;
    float prevHeadingVel;


    public static class WheelReversalPattern {
        final boolean horiz;
        final boolean vert;

        public WheelReversalPattern(boolean _horiz, boolean _vert) {
            horiz = _horiz;
            vert = _vert;
        }
    }

    public static class Wheels {
        public final DeadwheelWrapper horiz, vert;

        public Wheels(DeadwheelWrapper _horiz, DeadwheelWrapper _vert) {
            horiz = _horiz;
            vert = _vert;
        }

        @NonNull
        @Contract("_, _, _ -> new")
        public static Wheels fromMap(@NonNull HardwareMap.DeviceMapping<DcMotor> _map, String _horiz, String _vert) {
            return new Wheels(
                    new DeadwheelWrapper((DcMotorEx) _map.get(_horiz)),
                    new DeadwheelWrapper((DcMotorEx) _map.get(_vert))
            );

        }

        public Vec2 getPositions() {
            return new Vec2(vert.getPosition(), horiz.getPosition());
        }

        public Wheels reversalMap(@NonNull WheelReversalPattern pat) {
            horiz.setReverse(pat.horiz);
            vert.setReverse(pat.vert);

            return this;
        }
    }

    public static class WheelSpec {
        public final float ticksPerRev, radius;
        public final Vec2 vert, horiz;

        public final float distancePerTick;

        public WheelSpec(float _ticksPerRev, float _radius_cm, Vec2 _vert, Vec2 _horiz) {
            ticksPerRev = _ticksPerRev;
            radius = _radius_cm;
            vert = _vert;
            horiz = _horiz;

            distancePerTick = (float) (2 * Math.PI * radius / ticksPerRev);
        }
    }

    public static float normDelta(float angle, float prevAngle) {
        float delta = angle - prevAngle;

        while (delta >= Math.PI) delta -= (float) (2 * Math.PI);
        while (delta < -Math.PI) delta += (float) (2 * Math.PI);

        return prevAngle + delta;
    }

    @Override
    public void loop(float dt) {

        float rawHeading = (float) imu.getAngles().getYaw();
        float heading = normDelta(rawHeading, prevHeading);
        float headingVel = imu.getVelocity().zRotationRate;

        float dTheta = heading - prevHeading;
        float headingAcc = (headingVel - prevHeadingVel) / dt;
        Vec2 currEnc = wheels.getPositions();
        Vec2 dEnc = currEnc.sub(prevPos).scale(wheelSpec.distancePerTick);
        prevPos = currEnc;

        float dVert = dEnc.x;
        float dHoriz = dEnc.y;


        float dVert_rot = dTheta * (wheelSpec.vert.x);
        float dHoriz_rot = dTheta * (wheelSpec.horiz.x);

        float forward = dVert - dVert_rot;
        float strafe = dHoriz - dHoriz_rot;

        float cosH = (float) Math.cos(heading);
        float sinH = (float) Math.sin(heading);

        Vec2 dGlobal = new Vec2(
                forward * cosH - strafe * sinH,
                forward * sinH + strafe * cosH
        );

        Vec2 newPosXY = pos.asVec2().add(dGlobal);

        Vec2 newVelXY = dGlobal.scale(1f / dt);

        Vec2 newAccXY = newVelXY.sub(vel.asVec2()).scale(1f / dt);

        pos = new Vec2Rot(newPosXY, heading);
        vel = new Vec2Rot(newVelXY, headingVel);
        acc = new Vec2Rot(newAccXY, headingAcc);

        prevHeading = heading;
        prevHeadingVel = headingVel;
    }


    @Override
    public void setPos(Vec2Rot _pos) {
        pos = _pos;
    }

    @Override
    public Vec2Rot getPos() {
        return pos;
    }

    @Override
    public Vec2Rot getVel() {
        return vel;
    }

    @Override
    public Vec2Rot getAcc() {
        return acc;
    }

    @Override
    public void resetHeading() {
        pos.r = 0;
    }


    final IAction<Telemetry> telem = new WithTelemetry.Action<WithTelemetry.ITelemetry>(new WithTelemetry.ITelemetry() {
        @Override
        public String getName() {
            return "Three Wheel Odometry";
        }

        @Override
        public void loop(@NonNull Telemetry _telemetry) {
            _telemetry.addData("pos", pos.toString());
            _telemetry.addData("vel", vel.toString());
            _telemetry.addData("acc", acc.toString());
        }
    });

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telem;
    }

    public TwoWheelOdometry(WheelSpec _wheelSpec, Wheels _wheels, IIMU _imu) {
        pos = Vec2Rot.zero();
        vel = Vec2Rot.zero();
        acc = Vec2Rot.zero();
        imu = _imu;

        prevHeading = (float) imu.getAngles().getYaw();
        prevHeadingVel = imu.getVelocity().zRotationRate;

        wheelSpec = _wheelSpec;
        wheels = _wheels;

        prevPos = wheels.getPositions();
    }
}
