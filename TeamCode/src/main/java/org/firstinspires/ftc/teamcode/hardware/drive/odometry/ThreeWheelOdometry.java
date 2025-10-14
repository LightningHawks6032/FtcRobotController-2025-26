package org.firstinspires.ftc.teamcode.hardware.drive.odometry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.deadwheel.DeadwheelWrapper;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

public class ThreeWheelOdometry implements IOdometry{
    Vec2Rot pos, vel, acc;
    final WheelSpec wheelSpec;
    final Wheels wheels;

    Vec2Rot prevPos;

    public static class Wheels {
        public final DeadwheelWrapper left, right, back;

        public Wheels(DeadwheelWrapper _left, DeadwheelWrapper _right, DeadwheelWrapper _back) {
            left = _left;
            right = _right;
            back = _back;
        }

        public static Wheels fromMap(HardwareMap.DeviceMapping<DcMotor> _map, String _left, String _right, String _back) {

            return new Wheels(
                    new DeadwheelWrapper((DcMotorEx)_map.get(_left)),
                    new DeadwheelWrapper((DcMotorEx)_map.get(_right)),
                    new DeadwheelWrapper((DcMotorEx)_map.get(_back))
            );

        }

        public Vec2Rot getPositions() {
            return new Vec2Rot(left.getPosition(), right.getPosition(), back.getPosition());
        }
    }
    public static class WheelSpec {
        public final float ticksPerRev, radius;
        public final Vec2 leftWheel, rightWheel, backWheel;

        public final float distancePerTick;
        public final float lateralDistance;

        public WheelSpec(float _ticksPerRev, float _radius_cm, Vec2 _left, Vec2 _right, Vec2 _back) {
            ticksPerRev = _ticksPerRev;
            radius = _radius_cm;
            leftWheel = _left;
            rightWheel = _right;
            backWheel = _back;

            distancePerTick = (float)(2 * Math.PI * radius / ticksPerRev);
            lateralDistance = rightWheel.y - leftWheel.y;
        }
    }



    @Override
    public void loop(float dt) {

        Vec2Rot current = wheels.getPositions();
        Vec2Rot dEncPos = current.componentwiseSub(prevPos).componentwiseScl(wheelSpec.distancePerTick);
        prevPos = current;

        Vec2 dPosXY;
        float dTheta = (dEncPos.y - dEncPos.x) / wheelSpec.lateralDistance;
        if (Math.abs(dTheta) < 1e-6) {
             dPosXY = new Vec2(
                    (dEncPos.x + dEncPos.y) / 2f,
                    dEncPos.r
            );
        } else {
            float radius = (dEncPos.x + dEncPos.y) / (2f * dTheta);
            dPosXY = new Vec2(
                    (float) (radius * Math.sin(dTheta)),
                    (float) (radius * (1 - Math.cos(dTheta)) + dEncPos.r - wheelSpec.backWheel.x * dTheta)
            );
        }

        Vec2Rot dPos = new Vec2Rot(dPosXY, dTheta);

        float midHeading = pos.r + dPos.r / 2f;
        float cosH = (float) Math.cos(midHeading);
        float sinH = (float) Math.sin(midHeading);

        Vec2Rot dGlobalPos = new Vec2Rot(
                dPos.x * cosH - dPos.y * sinH,
                dPos.x * sinH + dPos.y * cosH,
                dPos.x
        );
        Vec2Rot newPos = pos.componentwiseAdd(dGlobalPos);

        float oneOverDt = 1f / dt;
        Vec2Rot newVel = dGlobalPos.componentwiseScl(oneOverDt);
        Vec2Rot newAcc = newVel.componentwiseSub(vel).componentwiseScl(oneOverDt);

        pos = newPos;
        vel = newVel;
        acc = newAcc;
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
            return "Mecanum Odometry";
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

    public ThreeWheelOdometry(WheelSpec _wheelSpec, Wheels _wheels) {
        pos = Vec2Rot.zero();
        vel = Vec2Rot.zero();
        acc = Vec2Rot.zero();

        wheelSpec = _wheelSpec;
        wheels = _wheels;
    }
}
