package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.SplitAction;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

import java.util.function.BiFunction;

/// Applies powers directly to the drive.<br>
/// `directDriveAction` - The direct drive action<br>
/// `splitAction` - splits the direct drive into stick-friendly components
public class DirectDrive {
    final DirectDriveAction directDriveAction;
    final SplitAction<Vec2, Vec2, Vec2Rot> splitAction;
    public IAction<Vec2Rot> directDriveAction() {return directDriveAction;}
    public SplitAction<Vec2, Vec2, Vec2Rot> splitAction() {return splitAction;}

    static class DirectDriveAction implements IAction<Vec2Rot> {
        DriveMotors drive;
        public DirectDriveAction(DriveMotors _drive) {
            drive = _drive;
        }

        Vec2Rot normalize(@NonNull Vec2Rot vec) {
            float d = Math.max(Math.abs(vec.x) + Math.abs(vec.y) + Math.abs(vec.r), 1);
            return new Vec2Rot(vec.x / d, vec.y / d, vec.r / d);
        }

        @Override
        public void init(RobotController robot, Vec2Rot data) {

        }

        @Override
        public void start(RobotController robot, Vec2Rot data) {

        }

        @Override
        public void loop(RobotController robot, Vec2Rot data) {

            Vec2Rot pow = normalize(data);

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
    }

    public DirectDrive(DriveMotors _drive, BiFunction<Vec2, Vec2, Vec2Rot> conversionMap) {
        directDriveAction = new DirectDriveAction(_drive);

        splitAction = new SplitAction<>(directDriveAction, conversionMap);
    }


}
