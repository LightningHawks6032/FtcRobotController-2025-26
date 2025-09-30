package org.firstinspires.ftc.teamcode.hardware.drive;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.DebugMotor;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.IMotorBuildOpt;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;

/**
 *
 * Robot birds-eye <br>
 *   ul------ur<br>
 *   |       |<br>
 *   |       |<br>
 *   |       |<br>
 *   dl-----dr
 */


public class DriveMotors {
    final IMotor ul;
    final IMotor ur;
    final IMotor dl;
    final IMotor dr;

    public IMotor ul() {return ul;}
    public IMotor ur() {return ur;}
    public IMotor dl() {return dl;}
    public IMotor dr() {return dr;}

    public DriveMotors(IMotor _ul, IMotor _ur, IMotor _dr, IMotor _dl) {
        ul = _ul;
        ur = _ur;
        dr = _dr;
        dl = _dl;
    }

    public static class Positions {
        public float ul, ur, dr, dl;

        public Positions(float _ul, float _ur, float _dr, float _dl) {
            setPositions(_ul, _ur, _dr, _dl);
        }

        public void setPositions(float _ul, float _ur, float _dr, float _dl) {
            ul = _ul;
            ur = _ur;
            dr = _dr;
            dl = _dl;
        }

        public Positions componentwiseSub(@NonNull Positions r) {
            return new Positions(ul - r.ul, ur - r.ur, dr - r.dr, dl - r.dl);
        }

        public Positions componentwiseScl(float scl) {
            return new Positions(ul * scl, ur * scl, dr * scl, dl * scl);
        }
    }

    public Positions getPositions() {
        return new Positions(
                ul.getPosition(),
                ur.getPosition(),
                dr.getPosition(),
                dl.getPosition()
        );
    }

    @NonNull
    public static DriveMotors fromMap(@NonNull HardwareMap.DeviceMapping<DcMotor> _map,
                                      @NonNull IMotorBuildOpt<? extends IMotor> _ul,
                                      @NonNull IMotorBuildOpt<? extends IMotor> _ur,
                                      @NonNull IMotorBuildOpt<? extends IMotor> _dr,
                                      @NonNull IMotorBuildOpt<? extends IMotor> _dl) {
        return new DriveMotors(
                _ul.fromMap(_map),
                _ur.fromMap(_map),
                _dr.fromMap(_map),
                _dl.fromMap(_map)
        );
    }

    @NonNull
    public static DriveMotors fromMapDcMotor(@NonNull HardwareMap.DeviceMapping<DcMotor> _map,
                                             boolean _usingEncoder, MotorSpec _motorSpec,
                                             String _ul,
                                             String _ur,
                                             String _dr,
                                             String _dl) {
        return new DriveMotors(
                new DcMotorWrapper.BuildOpt(_ul, _usingEncoder, _motorSpec).fromMap(_map),
                new DcMotorWrapper.BuildOpt(_ur, _usingEncoder, _motorSpec).fromMap(_map),
                new DcMotorWrapper.BuildOpt(_dr, _usingEncoder, _motorSpec).fromMap(_map),
                new DcMotorWrapper.BuildOpt(_dl, _usingEncoder, _motorSpec).fromMap(_map)
        );
    }

    // TODO: write telemetry wrapper
    @NonNull
    public static DriveMotors fromMapDebugMotor(@NonNull HardwareMap.DeviceMapping<DcMotor> _map,
                                                Telemetry _telemetry, MotorSpec _motorSpec,
                                                String _ul,
                                                String _ur,
                                                String _dr,
                                                String _dl) {
        return new DriveMotors(
                new DebugMotor.BuildOpt(_ul, _telemetry, _motorSpec).fromMap(_map),
                new DebugMotor.BuildOpt(_ur, _telemetry, _motorSpec).fromMap(_map),
                new DebugMotor.BuildOpt(_dr, _telemetry, _motorSpec).fromMap(_map),
                new DebugMotor.BuildOpt(_dl, _telemetry, _motorSpec).fromMap(_map)
        );
    }
}
