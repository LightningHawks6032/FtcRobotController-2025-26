package org.firstinspires.ftc.teamcode.opmodes.teleop.test;
import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.WaitAutoAction;
import org.firstinspires.ftc.teamcode.components.DirectDrive;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.components.action.PredicateAction;
import org.firstinspires.ftc.teamcode.control.IControlLoop;
import org.firstinspires.ftc.teamcode.control.IControlLoopBuildOpt;
import org.firstinspires.ftc.teamcode.hardware.GamepadWrapper;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.InputResponseManager;
import org.firstinspires.ftc.teamcode.hardware.InternalIMUWrapper;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.IOdometry;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.robot.ClankerHawk2A;
import org.firstinspires.ftc.teamcode.util.Pair;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

import java.util.function.BiFunction;
import java.util.function.Function;
//@Disabled // atp ts needs to be removed
@TeleOp(name = "Drive Motor Test", group = "Testing")
public class DriveMotorTestOpmode extends OpMode {
    static class MotorTestAction implements IAutoAction<ElapsedContainer> {

        final float duration;
        final IMotor motor;

        public MotorTestAction(IMotor _motor, float _duration) {
            duration = _duration;
            motor = _motor;
        }

        @Override
        public boolean isDone(float duration) {
            boolean done = duration >= this.duration;
            if (done) {
                motor.setPower(0); // TODO: Make an on stop event
            }
            return done;
        }

        @Override
        public Function<Pair<Float, IRobot>, ElapsedContainer> getDataProvider() {
            return _elapsed -> new ElapsedContainer(_elapsed.fst);
        }

        @Override
        public void init(IRobot robot, ElapsedContainer data) {

        }

        @Override
        public void start(IRobot robot, ElapsedContainer data) {

        }

        @Override
        public void loop(IRobot robot, ElapsedContainer data) {
            motor.setPower(1f);
        }
    }

    public static class DriveController {
        IControlLoop cx, cy, cr;

        public DriveController(@NonNull IControlLoopBuildOpt<? extends IControlLoop> _cx,
                               @NonNull IControlLoopBuildOpt<? extends IControlLoop> _cy,
                               @NonNull IControlLoopBuildOpt<? extends IControlLoop> _cr) {
            cx = _cx.build();
            cy = _cy.build();
            cr = _cr.build();
        }

        public DriveController(@NonNull IControlLoopBuildOpt<? extends  IControlLoop> _cp,
                               @NonNull IControlLoopBuildOpt<? extends IControlLoop> _cr) {
            cx = _cp.build();
            cy = _cp.build();
            cr = _cr.build();
        }


        public Vec2Rot loop(@NonNull Vec2Rot current, Vec2Rot target, float dt) {
            return new Vec2Rot(
                    cx.loop(current.x, current.x, dt),
                    cy.loop(current.y, current.y, dt),
                    cr.loop(current.r, current.r, dt)
            );
        }
    }

    public static class IMUTest {
        IMU imu;

        public IMU imu() {return imu;}

        public IMUTest(IMU _imu) {
            imu = _imu;

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu.initialize(new IMU.Parameters(orientationOnRobot));
        }

        IAction<Telemetry> telemetryAction = WithTelemetry.fromLambda(() -> "IMU", (telem) -> {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            telem.addData("Yaw", angles.getYaw(AngleUnit.DEGREES));
            telem.addData("Pitch", angles.getPitch(AngleUnit.DEGREES));
            telem.addData("Roll", angles.getRoll(AngleUnit.DEGREES));
            //testicl
        });

        public IAction<Telemetry> getTelemetryAction() {
            return telemetryAction;
        }
    }

    /// Provides three actions, the setters for `Ty1` and `Ty2` and the loop action that implements the split action
    @NonNull
    static AutoActionSequence<ElapsedContainer> testDrive(@NonNull DriveMotors drive) {
        return new AutoActionSequence<>(
            new MotorTestAction(drive.ul(), 1f),
            new WaitAutoAction(1f),
            new MotorTestAction(drive.ur(), 1f),
            new WaitAutoAction(1f),
            new MotorTestAction(drive.dr(), 1f),
            new WaitAutoAction(1f),
            new MotorTestAction(drive.dl(), 1f)
        );
    }

    InternalIMUWrapper imu;

    DriveMotors drive;
    LaunchAutoSequenceAction<ElapsedContainer> actionExecutor;
    IRobot robot;
    DirectDrive directDrive;
    InputResponseManager inputResponseManager;
    IOdometry odometry;
    ElapsedTime timer;
    BiFunction<Vec2, Vec2, Vec2Rot> usingFieldCentric;
    @Override
    public void init() {
        robot = new ClankerHawk2A(hardwareMap);
        imu = new InternalIMUWrapper(hardwareMap.get(IMU.class, "imu"));

        drive = DriveMotors.fromMapDcMotor(hardwareMap.dcMotor, true, MotorSpec.GOBILDA_5203_2402_0019,
                "fl", "fr", "br", "bl"
        );

        directDrive = new DirectDrive(drive);

        drive.ur().setDirection(IMotor.Direction.REVERSE);
        drive.dl().setDirection(IMotor.Direction.FORWARD);
        drive.dr().setDirection(IMotor.Direction.REVERSE);
        drive.ul().setDirection(IMotor.Direction.FORWARD);

        actionExecutor = new LaunchAutoSequenceAction<>(
                testDrive(drive)
        );

//        SplitAction<Vec2, Vec2, Vec2Rot> driveAction = new SplitAction<>(
//                new DirectDrive(drive),
//                (v1, r) -> {
//                    return usingFieldCentric.apply(v1, r.x);}
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
                ThreeWheelOdometry.Wheels.fromMap(hardwareMap.dcMotor, "fl", "fr", "bl")
                        .reversalMap(
                                new ThreeWheelOdometry.WheelReversalPattern(
                                        false,
                                        true,
                                        false
                                )
                        )
        );

//        odometry = new MecanumOdometry(drive, new MecanumOdometry.WheelSpec(
//                7,
//                MotorSpec.GOBILDA_5203_2402_0003.encoderResolution * MotorSpec.GOBILDA_5203_2402_0003.gearRatio,
//                20,
//                (float)Math.PI / 4f
//        ));

        inputResponseManager = new InputResponseManager.Builder(new GamepadWrapper(gamepad1), robot, telemetry)
                .AAction(actionExecutor)
                .leftStickAction(directDrive.splitAction().leftSetter())
                .rightStickAction(directDrive.splitAction().rightSetter())
                .loops(new PredicateAction<>(new EmptyAction<>(), directDrive.splitAction(), (robot, v) -> actionExecutor.running()))
                .telemetry(
                        odometry.getTelemetryAction(),
                        drive.getTelemetryAction(),
                        imu.getTelemetryAction()
                )
                .build();


        timer = new ElapsedTime();


    }

    @Override
    public void start() {
        timer.reset();
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
