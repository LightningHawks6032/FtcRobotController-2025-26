package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.components.DirectDrive;
import org.firstinspires.ftc.teamcode.components.FlywheelController;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.IntakeMotorController;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.MapAction;
import org.firstinspires.ftc.teamcode.components.action.PredicateAction;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.InternalIMUWrapper;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;
import org.firstinspires.ftc.teamcode.hardware.ServoWrapper;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.hardware.drive.IIMU;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.IOdometry;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.opmodes.teleop.test.DriveMotorTestOpmode;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

/// Test drivetrain
public class ClankerHawk2A implements IRobot {
    /*
    --- Robot Mapping ---
    c - control hub, e - expansion hub
    m - motor port, s - servo port

    cm[0]
    cm[1] - intake
    cm[2]
    cm[3]

    cs[0]
    cs[1]
    cs[2]
    cs[3]
    cs[4]

    em[0]
    em[1]
    em[2]
    em[3]

    es[0]
    es[1]
    es[2]
    es[3]
    es[4]
 */
    final DriveMotors driveMotors;
    final IOdometry odometry;
    final IIMU imu;
    final DriveMotorTestOpmode.DriveController driveController;
    public final DirectDrive directDrive;
    public final FlywheelController flywheelController;
    public final IntakeMotorController intakeController;
    public final IAction<Boolean> resetHeadingAction;


    @Override
    public DriveMotors getDrive() {
        return driveMotors;
    }

    @Override
    public IOdometry getOdometry() {
        return odometry;
    }

    @Override
    public IIMU getIMU() {
        return imu;
    }

    public ClankerHawk2A(@NonNull HardwareMap hardwareMap) {
        driveMotors = DriveMotors.fromMapDcMotor(hardwareMap.dcMotor,
                false,
                MotorSpec.GOBILDA_5203_2402_0019,
                "fl", "fr", "br", "bl"
        );
        driveMotors.ur().setDirection(IMotor.Direction.REVERSE);
        driveMotors.dl().setDirection(IMotor.Direction.FORWARD);
        driveMotors.dr().setDirection(IMotor.Direction.REVERSE);
        driveMotors.ul().setDirection(IMotor.Direction.FORWARD);

        driveMotors.setEncoderPositionSignMap(new DriveMotors.EncoderPositionSignMap(
                false,
                false,
                false,
                false
        ));

        driveController = new DriveMotorTestOpmode.DriveController(
                new PIDF.BuildOpt(
                        new PIDF.Weights(0.0035f, 0f, 0.1f, 1, 0.3f, 0.1f)
                ),
                new PIDF.BuildOpt(
                        new PIDF.Weights(0.0035f, 0f, 0.1f, 1, 0.3f, 0.1f)
                )
        );

        odometry = new ThreeWheelOdometry(
                new ThreeWheelOdometry.WheelSpec(
                        2000,
                        1.6f,
                        new Vec2(0, -1.6f),
                        new Vec2(0, 1.6f),
                        new Vec2(0.6f, 0)
                ),
                ThreeWheelOdometry.Wheels.fromMap(hardwareMap.dcMotor, "fr", "bl", "br")
                        .reversalMap(
                                new ThreeWheelOdometry.WheelReversalPattern(
                                        true,
                                        false,
                                        false
                                )
                        )
        );

        imu = new InternalIMUWrapper(hardwareMap.get(IMU.class, "imu"));

        directDrive = new DirectDrive(driveMotors, DirectDrive.fieldCentricFromIMU(imu));

        flywheelController = new FlywheelController(
                Util.also(
                        new DcMotorWrapper(hardwareMap.dcMotor.get("flywheel"), true, MotorSpec.GOBILDA_5000_0002_0001),
                        f -> f.setDirection(IMotor.Direction.REVERSE)
                ),
                new ServoWrapper(hardwareMap.servo.get("hood"))
        );

        intakeController = new IntakeMotorController(
                new DcMotorWrapper(hardwareMap.dcMotor.get("intake"), true, MotorSpec.GOBILDA_5203_2402_0003)
        );

        resetHeadingAction = new PredicateAction<>(
                IAction.From.loop((r, b) -> imu.resetYaw()),
                new EmptyAction<>(),
                (r, b) -> b
        );

    }
}
