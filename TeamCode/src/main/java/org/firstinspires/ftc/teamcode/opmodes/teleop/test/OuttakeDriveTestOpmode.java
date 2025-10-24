package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.DirectDrive;
import org.firstinspires.ftc.teamcode.components.FlywheelController;
import org.firstinspires.ftc.teamcode.components.RobotController;
import org.firstinspires.ftc.teamcode.components.action.SplitAction;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.GamepadWrapper;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.InputResponseManager;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.IOdometry;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;




@TeleOp(name = "Outtake Drive Test", group = "Testing")
public class OuttakeDriveTestOpmode extends OpMode {

    DriveMotors drive;
    RobotController robot;
    DirectDrive directDrive;
    FlywheelController flywheelController;
    InputResponseManager inputResponseManager;
    IOdometry odometry;
    ElapsedTime timer;
    DriveMotorTestOpmode.IMUTest imu;

    @Override
    public void init() {
        robot = new RobotController();
        drive = DriveMotors.fromMapDcMotor(hardwareMap.dcMotor, true, MotorSpec.GOBILDA_5203_2402_0019,
                "fl", "fr", "br", "bl"
        );
        imu = new DriveMotorTestOpmode.IMUTest(hardwareMap.get(IMU.class, "imu"));
        directDrive = new DirectDrive(drive,
                (v, r) ->  new Vec2Rot(v.rotateOrigin((float) imu.imu().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)), r.x)
        );
        flywheelController = new FlywheelController(
                new DcMotorWrapper(
                        hardwareMap.dcMotor.get("flywheel"),
                        true,
                        MotorSpec.GOBILDA_5000_0002_0001
                )
        );

        drive.ur().setDirection(IMotor.Direction.REVERSE);
        drive.dl().setDirection(IMotor.Direction.FORWARD);
        drive.dr().setDirection(IMotor.Direction.REVERSE);
        drive.ul().setDirection(IMotor.Direction.FORWARD);

//        SplitAction<Vec2, Vec2, Vec2Rot> driveAction = new SplitAction<>(
//                new DirectDrive(drive),
//                (v1, r) -> {
//                    return new Vec2Rot(v1, r.x);}//usingFieldCentric.apply(v1, r.x);}
//        );

        drive.setEncoderPositionSignMap(new DriveMotors.EncoderPositionSignMap(
                false,
                false,
                false,
                false
        ));
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
        inputResponseManager = new InputResponseManager.Builder(new GamepadWrapper(gamepad1), robot, telemetry)
                .leftStickAction(directDrive.splitAction().leftSetter())
                .rightStickAction(directDrive.splitAction().rightSetter())
                .loops(
                        directDrive.splitAction(),
                        flywheelController.setMotorPowerAction()
                )
                .rightTriggerAction(flywheelController.setPowerAction())
                .AAction(flywheelController.setLockAction())
                .telemetry(
                        odometry.getTelemetryAction(),
                        drive.getTelemetryAction(),
                        flywheelController.getTelemetryAction(),
                        imu.getTelemetryAction()
                )
                .build();


        timer = new ElapsedTime();

    }

    @Override
    public void loop() {
        float dt = (float)timer.seconds();
        if (dt < 1e-6) {dt = 1e-6f;}
        odometry.loop(dt);
        timer.reset();

        inputResponseManager.loop();

        if (gamepad1.x) {
            odometry.resetHeading();
        }
    }
}
