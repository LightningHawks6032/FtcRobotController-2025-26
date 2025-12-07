package org.firstinspires.ftc.teamcode.opmodes.teleop.test;
import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.WaitAutoAction;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.action.AxisSplitterAction;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.components.action.PredicateAction;
import org.firstinspires.ftc.teamcode.control.IControlLoop;
import org.firstinspires.ftc.teamcode.control.IControlLoopBuildOpt;
import org.firstinspires.ftc.teamcode.hardware.GamepadWrapper;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.InputResponseManager;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.Pair;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

import java.util.function.Function;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name = "Drive Motor Test", group = "Testing") //fortnite
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
    AutoActionSequence<ElapsedContainer> testDrive(@NonNull DriveMotors drive) {
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

    LaunchAutoSequenceAction<ElapsedContainer> actionExecutor;
    ThunderclapRobot robot;
    InputResponseManager inputResponseManager, player2;
    Servo outtakeServo;
    DcMotor intakeServo;
    ElapsedTime timer;
    IAction<Float> intakeServoAction;
    @Override
    public void init() {
        robot = new ThunderclapRobot(hardwareMap);
        actionExecutor = new LaunchAutoSequenceAction<>(
                testDrive(robot.driveMotors)
        );
        intakeServoAction = IAction.From.loop((r, p) -> {
            outtakeServo.setPosition((p + 1)/2f);
        });
        outtakeServo = hardwareMap.servo.get("transfer flywheel");
        intakeServo = hardwareMap.dcMotor.get("intake flywheel");
        inputResponseManager = new InputResponseManager.Builder(new GamepadWrapper(gamepad1), robot, telemetry)
                .AAction(actionExecutor)


                .leftStickAction(robot.directDrive.splitAction().leftSetter(),
                        IAction.From.loop(
                                (r, f) -> {
                                    if (f.y < -0.3f) intakeServoAction.loop(r, 1f);
                                })
                        )
                .rightStickAction(robot.directDrive.splitAction().rightSetter())
                .leftBumperAction(
                        robot.directDrive.slowModeAction()
                )
                .rightBumperAction(
                        robot.directDrive.fastModeAction()
                )
                .loops(new PredicateAction<>(new EmptyAction<>(), robot.directDrive.splitAction(), (robot, v) -> actionExecutor.running()))
                .telemetry(
                        robot.getOdometry(),
                        robot.getDrive(),
                        robot.getIMU()
                )
                .timeLoops(
                        robot.getOdometry().getLoopAction()
                )
                .XAction(
                        robot.resetHeadingAction
                )
                .build();
        /*player2 = new InputResponseManager.Builder(new GamepadWrapper(gamepad2), robot, telemetry)
                .rightTriggerAction(
                        //robot.flywheelController.setPowerAction()
                )
                .BAction(
                        //robot.flywheelController.setLockAction()
                )
                .DPadAction(
                        AxisSplitterAction.TwoWay(
                                robot.flywheelController.setHoodPowerAction(),
                                intakeServoAction
                        )
                )
                .leftStickAction(
                        AxisSplitterAction.TwoWay(
                                new EmptyAction<>(),
                                IAction.From.loop((r, p) -> {
                                    intakeServo.setPower(-p);
                                    //i likeh kids i am marcello i am marcello and i have pp                               })
                                })
                        )
                )
                .loops(
                        robot.flywheelController.setMotorPowerAction()
                )
                .build();*/

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
        timer.reset();

        inputResponseManager.loop();
        //player2.loop();
    }
}
