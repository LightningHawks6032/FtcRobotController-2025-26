package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.DirectDrive;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
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
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ThunderclapRobot implements IRobot {

    final IOdometry odometry;
    final IIMU imu;
    public final DriveMotors driveMotors;

    public final DirectDrive directDrive;
    public final StateMachineDrive stateMachineDrive;
    public final OuttakeWheelController outtakeController;

    public final IntakeWheelController intakeController;

    public final TransferWheelController transferController;

    public final HoodController hoodController;

    public final InternalCameraWrapper camera;

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

    public ThunderclapRobot(@NonNull HardwareMap hardwareMap) {
        imu = new InternalIMUWrapper(hardwareMap.get(IMU.class, "imu"));

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

        odometry = new TwoWheelOdometry(
                new TwoWheelOdometry.WheelSpec(
                        2000,
                        2.4f,
                        new Vec2(0, -9), //1.6
                        //new Vec2(0, -9),
                        new Vec2(-1f, 18) //0.6
                ),
                TwoWheelOdometry.Wheels.fromMap(hardwareMap.dcMotor, "fl", "fr")
                        .reversalMap(
                                new TwoWheelOdometry.WheelReversalPattern(
                                        true,
                                        true
                                )
                        ),
                getIMU()
        );

        directDrive = new DirectDrive(driveMotors, DirectDrive.fieldCentricFromIMUGamepad(imu));
        directDrive.setDrivePowerFactor(0.8f);


        camera = new InternalCameraWrapper(hardwareMap);
        stateMachineDrive = new StateMachineDrive(directDrive, new PIDF.BuildOpt(new PIDF.Weights(
                0.8f, 0f,0.05f,0,0,1
        )), () -> {
            float fallback = 0f;
            AprilTagDetection last = camera.lastReading;
            if (last == null) {return fallback;}
            return (float)last.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
        }, () -> (float)imu.getAngles().getYaw(AngleUnit.RADIANS));

        outtakeController = new OuttakeWheelController(
                Util.also(new DcMotorWrapper(hardwareMap.dcMotor.get("outtake flywheel"), true, MotorSpec.GOBILDA_5000_0002_0001),
                        m->m.setDirection(IMotor.Direction.REVERSE)),
                    new PIDF.BuildOpt(new PIDF.Weights(
                            0.9f,
                            0.7f,0.25f,
                            1f,
                            0.1f,1
                    )),
                () -> camera.lastReading
                );

        hoodController = new HoodController(
                new ServoWrapper(hardwareMap.servo.get("hood")),
                0.4f, 0.8f,
                () -> {
                    AprilTagDetection last = camera.lastReading;
                    if (last == null) {
                        return 100f;
                    }
                    return (float)last.ftcPose.range;
                }
        );

        intakeController = new IntakeWheelController(
                new DcMotorWrapper(hardwareMap.dcMotor.get("intake flywheel"), false, MotorSpec.GOBILDA_5203_2402_0019)
        );

        transferController = new TransferWheelController(
                new ServoWrapper(hardwareMap.servo.get("transfer flywheel")),
                intakeController::trySyncTransferLift
        );

        resetHeadingAction = new PredicateAction<>(
                IAction.From.loop((r, b) -> imu.resetYaw()),
                new EmptyAction<>(),
                (r, b) -> b
        );
    }
}
