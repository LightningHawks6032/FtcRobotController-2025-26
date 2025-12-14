package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.SplitAction;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.hardware.drive.IIMU;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.IOdometry;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.jetbrains.annotations.Contract;

import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

/// Applies powers directly to the drive.<br>
/// `directDriveAction` - The direct drive action<br>
/// `splitAction` - splits the direct drive into stick-friendly components
public class DirectDrive {
    final DirectDriveAction directDriveAction;
    final SplitAction<Vec2, Vec2, Vec2Rot> splitAction;
    final SpeedModeAction speedModeAction;
    public IAction<Vec2Rot> directDriveAction() {return directDriveAction;}
    public SplitAction<Vec2, Vec2, Vec2Rot> splitAction() {return splitAction;}
    public IAction<Boolean> fastModeAction() {return speedModeAction.fastModeAction;}
    public IAction<Boolean> slowModeAction() {return speedModeAction.slowModeAction;}
    public void setDrivePowerFactor(float newFactor) {
        directDriveAction.drivePowerFactor = newFactor;
    }

    public Vec2Rot getLastStickInput() {
        return Vec2Rot.zero();
        //Vec2Rot last = splitAction.getLast();
        //return last == null ? Vec2Rot.zero() : last;
    }

    float getDrivePowerFactor() {
        return directDriveAction.drivePowerFactor;
    }

    static class DirectDriveAction implements IAction<Vec2Rot> {
        DriveMotors drive;
        float drivePowerFactor = 1f;
        public DirectDriveAction(DriveMotors _drive) {
            drive = _drive;
        }

        Vec2Rot normalize(@NonNull Vec2Rot vec) {
            float d = Math.max(Math.abs(vec.x) + Math.abs(vec.y) + Math.abs(vec.r), 1);
            return new Vec2Rot(vec.x / d, vec.y / d, vec.r / d);
        }

        @Override
        public void init(IRobot robot, Vec2Rot data) {

        }

        @Override
        public void start(IRobot robot, Vec2Rot data) {

        }

        @Override
        public void loop(IRobot robot, Vec2Rot data) {

            Vec2Rot pow = normalize(data).componentwiseScl(drivePowerFactor);

            drive.setPower(
                    (pow.y - pow.x - pow.r),
                    (pow.y + pow.x + pow.r),
                    (pow.y - pow.x + pow.r),
                    (pow.y + pow.x - pow.r)
            );
        }
    }

    public DirectDrive(DriveMotors _drive) {
        directDriveAction = new DirectDriveAction(_drive);

        splitAction = new SplitAction<>(directDriveAction, (v1, r) -> new Vec2Rot(v1, r.x));

        speedModeAction = new SpeedModeAction(this::setDrivePowerFactor);

    }

    public DirectDrive(DriveMotors _drive, BiFunction<Vec2, Vec2, Vec2Rot> conversionMap) {
        directDriveAction = new DirectDriveAction(_drive);

        splitAction = new SplitAction<>(directDriveAction, conversionMap);

        speedModeAction = new SpeedModeAction(this::setDrivePowerFactor);
    }

    @NonNull
    @Contract(pure = true)
    public static BiFunction<Vec2, Vec2, Vec2Rot> fieldCentricFromIMUGamepad(IIMU _imu) {
        return ((BiFunction<Vec2,Vec2,Vec2Rot>)(v1, v2) -> new Vec2Rot(v1.x, v1.y, v2.x)).andThen(fieldCentricFromIMU(_imu));
    }
    @NonNull
    @Contract(pure = true)
    public static Function<Vec2Rot, Vec2Rot> fieldCentricFromIMU(IIMU _imu) {
        return (v) -> new Vec2Rot(v.asVec2().rotateOrigin((float) _imu.getAngles().getYaw(AngleUnit.RADIANS)), v.r);
    }

    @NonNull
    @Contract(pure = true)
    public static BiFunction<Vec2, Vec2, Vec2Rot> fieldCentricFromOdometryGamepad(IOdometry _odo) {
        return (v1, v2) -> new Vec2Rot(v1.rotateOrigin(_odo.getPos().r), v2.x);
    }


    static class SpeedModeAction {
        enum SPEED {
            NORMAL(0.8f), SLOW(0.3f), FAST(1f);

            public float speed;

            SPEED(float _speed) {
                speed = _speed;
            }
        }


        Consumer<Float> updateSpeed;
        SPEED state;

        IAction<Boolean> fastModeAction, slowModeAction;

        public void setSpeeds(float slow, float normal, float fast) {
            SPEED.NORMAL.speed = normal;
            SPEED.FAST.speed = fast;
            SPEED.SLOW.speed = slow;
        }
        void updateState(SPEED _speed) {
            if (_speed == state) {return;}
            state = _speed;
            updateSpeed.accept(state.speed);
        }

        public SpeedModeAction(Consumer<Float> _updateSpeed) {
            state = SPEED.NORMAL;
            updateSpeed = _updateSpeed;

            fastModeAction = IAction.From.loop((r, b) -> {
                if (b && state == SPEED.NORMAL) {
                    updateState(SPEED.FAST);
                }
                else if (!b && state==SPEED.FAST) {
                    updateState(SPEED.NORMAL);
                }
            });

            slowModeAction = IAction.From.loop((r, b) -> {
                if (b && state == SPEED.NORMAL) {
                    updateState(SPEED.SLOW);
                }
                else if (!b && state==SPEED.SLOW) {
                    updateState(SPEED.NORMAL);
                }            });
        }
    }
}
