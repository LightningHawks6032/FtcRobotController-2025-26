package org.firstinspires.ftc.teamcode.hardware.drive.odometry;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

public class MecanumOdometry implements IOdometry {
    Vec2Rot pos, vel, acc;
    final WheelSpec wheelSpec;
    DriveMotors.Positions posReadings;
    final DriveMotors drive;

    final IAction<Telemetry> telem = new WithTelemetry.Action<WithTelemetry.ITelemetry>(new WithTelemetry.ITelemetry() {
        @Override
        public String getName() {
            return "Mecanum Drive";
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

    public static class WheelSpec {
        public final float radius, ticksPerRev, displacement, angleOffPerp;

        ///  cm
        public final float distancePerTick;
        public final float xDisplacement;

        public WheelSpec(float _radius_cm, float _ticksPerRev, float _displacement_cm, float _angleOffPerp_rad) {
            radius = _radius_cm;
            ticksPerRev = _ticksPerRev;
            displacement = _displacement_cm;
            angleOffPerp = _angleOffPerp_rad;

            distancePerTick = 2 * (float)Math.PI  * radius / ticksPerRev;
            xDisplacement = displacement * (float)Math.cos(angleOffPerp);
        }
    }


    public MecanumOdometry(DriveMotors _drive, WheelSpec _wheelSpec) {
        pos = Vec2Rot.zero();
        vel = Vec2Rot.zero();
        acc = Vec2Rot.zero();

        wheelSpec = _wheelSpec;
        drive = _drive;
        posReadings = drive.getPositions();
    }


    @Override
    public void loop(float dt) {
        DriveMotors.Positions current = drive.getPositions();

        current.dr = -current.dr;
        current.ur = -current.ur;

        DriveMotors.Positions dEncPos = current
                .componentwiseSub(posReadings)
                .componentwiseScl(wheelSpec.distancePerTick);


        posReadings = current;

        Vec2Rot dPos = new Vec2Rot(
                dEncPos.ul + dEncPos.ur + dEncPos.dr + dEncPos.dl,
                dEncPos.ul + dEncPos.dr - dEncPos.ur - dEncPos.dl,
                (dEncPos.ur + dEncPos.dr - dEncPos.ul - dEncPos.dl) *
                        wheelSpec.xDisplacement
        ).componentwiseScl(1f/4);

        float cosH = (float)Math.cos(pos.r);
        float sinH = (float)Math.sin(pos.r);
        Vec2Rot dGlobalPos = new Vec2Rot(
                dPos.x * cosH - dPos.y * sinH,
                dPos.x * sinH + dPos.y * cosH,
                dPos.r
        );

        Vec2Rot newPos = pos.componentwiseAdd(dGlobalPos);

        float one_over_dt = 1f/dt;
        Vec2Rot newVel = dGlobalPos.componentwiseScl(one_over_dt);
        Vec2Rot newAcc = newVel.componentwiseSub(vel).componentwiseScl(one_over_dt);

        pos = newPos;
        vel = newVel;
        acc = newAcc;
    }

    @Override
    public void setPos(Vec2Rot _pos) {
        pos = _pos;
        vel = Vec2Rot.zero();
        acc = Vec2Rot.zero();
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
}
