package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.WaitAutoAction;
import org.firstinspires.ftc.teamcode.components.RobotController;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.hardware.GamepadWrapper;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.InputResponseManager;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.IOdometry;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.MecanumOdometry;
import org.firstinspires.ftc.teamcode.util.Pair;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

import java.util.function.BiFunction;
import java.util.function.Function;

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
        public Function<Pair<Float, RobotController>, ElapsedContainer> getDataProvider() {
            return _elapsed -> new ElapsedContainer(_elapsed.fst);
        }

        @Override
        public void init(RobotController robot, ElapsedContainer data) {

        }

        @Override
        public void start(RobotController robot, ElapsedContainer data) {

        }

        @Override
        public void loop(RobotController robot, ElapsedContainer data) {
            motor.setPower(1f);
        }
    }

    static class PredicateAction <Ty> implements IAction<Ty> {

        IAction<Ty> onTrue;
        IAction<Ty> onFalse;
        BiFunction<RobotController, Ty, Boolean> pred;


        public PredicateAction(IAction<Ty> _onTrue) {
            onTrue = _onTrue;
            onFalse = new EmptyAction<>();
        }

        public PredicateAction(IAction<Ty> _onTrue, IAction<Ty> _onFalse, BiFunction<RobotController, Ty, Boolean> _pred) {
            onTrue = _onTrue;
            onFalse = _onFalse;
            pred = _pred;
        }

        IAction<Ty> getRunningAction(boolean pred) {
            return pred ? onTrue : onFalse;
        }

        @Override
        public void init(RobotController robot, Ty data) {
            getRunningAction(pred.apply(robot, data)).init(robot, data);
        }

        @Override
        public void start(RobotController robot, Ty data) {
            getRunningAction(pred.apply(robot, data)).start(robot, data);
        }

        @Override
        public void loop(RobotController robot, Ty data) {
            getRunningAction(pred.apply(robot, data)).loop(robot, data);
        }
    }

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

            drive.ul().setPower(pow.y - pow.x - pow.r);
            drive.ur().setPower(pow.y + pow.x + pow.r);
            drive.dl().setPower(pow.y + pow.x - pow.r);
            drive.dr().setPower(pow.y - pow.x + pow.r);
        }
    }

    /// Provides three actions, the setters for `Ty1` and `Ty2` and the loop action that implements the split action
    static class SplitAction <Ty1, Ty2, TySplit> implements IAction<Object> {
        IAction<Ty1> leftSetter;
        IAction<Ty2> rightSetter;
        IAction<TySplit> action;

        public IAction<Ty1> leftSetter() {return leftSetter;}
        public IAction<Ty2> rightSetter() {return rightSetter;}

        Ty1 leftData;
        Ty2 rightData;

        BiFunction<Ty1, Ty2, TySplit> conversionMap;

        class LeftSetterAction implements IAction<Ty1>{

            @Override
            public void init(RobotController robot, Ty1 data) {
                leftData = data;
            }

            @Override
            public void start(RobotController robot, Ty1 data) {
                leftData = data;
            }

            @Override
            public void loop(RobotController robot, Ty1 data) {
                leftData = data;
            }
        }
        class RightSetterAction implements IAction<Ty2>{

            @Override
            public void init(RobotController robot, Ty2 data) {
                rightData = data;
            }

            @Override
            public void start(RobotController robot, Ty2 data) {
                rightData = data;
            }

            @Override
            public void loop(RobotController robot, Ty2 data) {
                rightData = data;
            }
        }

        public SplitAction(IAction<TySplit> _action, BiFunction<Ty1, Ty2, TySplit> _conversionMap) {
            action = _action;
            conversionMap = _conversionMap;
            leftSetter = new LeftSetterAction();
            rightSetter = new RightSetterAction();
        }

        @Override
        public void init(RobotController robot, Object data) {
            action.init(robot, conversionMap.apply(leftData, rightData));
        }

        @Override
        public void start(RobotController robot, Object data) {
            action.start(robot, conversionMap.apply(leftData, rightData));
        }

        @Override
        public void loop(RobotController robot, Object data) {
            action.loop(robot, conversionMap.apply(leftData, rightData));
        }
    }

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

    DriveMotors drive;
    LaunchAutoSequenceAction<ElapsedContainer> actionExecutor;
    RobotController robot;
    InputResponseManager inputResponseManager;
    IOdometry odometry;
    ElapsedTime timer;

    BiFunction<Vec2, Float, Vec2Rot> usingFieldCentric;
    @Override
    public void init() {
        robot = new RobotController();
        drive = DriveMotors.fromMapDcMotor(hardwareMap.dcMotor, true, MotorSpec.GOBILDA_5203_2402_0019,
                "lf", "rf", "rr", "lr"
        );

        drive.ur().setDirection(IMotor.Direction.REVERSE);
        drive.dl().setDirection(IMotor.Direction.FORWARD);
        drive.dr().setDirection(IMotor.Direction.REVERSE);
        drive.ul().setDirection(IMotor.Direction.FORWARD);


        actionExecutor = new LaunchAutoSequenceAction<>(
                testDrive(drive)
        );

        SplitAction<Vec2, Vec2, Vec2Rot> driveAction = new SplitAction<>(
                new DirectDriveAction(drive),
                (v1, r) -> {

                    telemetry.addData("rotation", r.x);
                    return new Vec2Rot(v1, r.x);}
        );

        odometry = new MecanumOdometry(drive, new MecanumOdometry.WheelSpec(
                7,
                MotorSpec.GOBILDA_5203_2402_0003.encoderResolution * MotorSpec.GOBILDA_5203_2402_0003.gearRatio,
                1,
                (float)Math.PI / 4f
        ));

        inputResponseManager = new InputResponseManager.Builder(new GamepadWrapper(gamepad1), robot, telemetry)
                .AAction(actionExecutor)
                .leftStickAction(driveAction.leftSetter)
                .rightStickAction(driveAction.rightSetter)
                .loops(new PredicateAction<>(new EmptyAction<>(), driveAction, (robot, v) -> actionExecutor.running()))
                .telemetry(
                        odometry.getTelemetryAction()
                )
                .build();


        timer = new ElapsedTime();

        usingFieldCentric = (v, r) -> new Vec2Rot(v.rotateOrigin(-odometry.getPos().r), r);;
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
    }
}
