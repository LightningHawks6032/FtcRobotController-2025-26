package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IActionAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.IAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.WaitAutoAction;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.ObeliskPattern;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;
import org.firstinspires.ftc.teamcode.hardware.ServoWrapper;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.SpindexerController;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.Pair;
import org.firstinspires.ftc.teamcode.util.TimerWrapper;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

import java.util.function.Function;

@Autonomous(name="Close Red")
public class CloseRedOpmode extends OpMode {
    public class SimpleBackwardTravel implements IAutoAction<ElapsedContainer> {

        final static float TRANS_THRESHOLD = 5f;
        float target;

        @Override
        public boolean isDone(float duration) {
            float diff = robot.getOdometry().getPos().x - target ;
            return Math.abs(diff) <= TRANS_THRESHOLD;
        }

        @Override
        public Function<Pair<Float, IRobot>, ElapsedContainer> getDataProvider() {
            return _elapsed -> new ElapsedContainer(_elapsed.fst);
        }

        public SimpleBackwardTravel(float _target) {
            target = _target;
        }

        @Override
        public void init(IRobot robot, ElapsedContainer data) {

        }

        @Override
        public void start(IRobot robot, ElapsedContainer data) {

        }

        @Override
        public void loop(IRobot _robot, ElapsedContainer data) {
            Vec2Rot pos = robot.getOdometry().getPos();
            Vec2Rot pow = new Vec2Rot(new Vec2(
                    0,
                    Math.signum(target - pos.x)
            ).norm().scale(0.3f)/*.rotateOrigin((float)robot.getIMU().getAngles().getYaw())*/, 0);

            telemetry.addData("power", pow.toString());
            robot.directDrive.directDriveAction().loop(robot,pow);
        }
    }

    static class ActionAutoAction <T> implements IAutoAction<ElapsedContainer> {

        final float duration;
        final IAction<T> action;
        final Function<ElapsedContainer, T> dataProvider;

        public ActionAutoAction(float _duration, IAction<T> _action, Function<ElapsedContainer, T> _dataProvider) {
            duration = _duration;
            action = _action;
            dataProvider = _dataProvider;
        }

        @Override
        public boolean isDone(float duration) {
            boolean done = duration >= this.duration;
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
            action.loop(robot, dataProvider.apply(data));
        }
    }

    public class SimpleRotTravel implements IAutoAction<ElapsedContainer> {

        final static float TRANS_THRESHOLD = 0.02f;
        float target;

        @Override
        public boolean isDone(float duration) {
            float diff = robot.getOdometry().getPos().r - target ;
            return Math.abs(diff) <= TRANS_THRESHOLD;
        }

        @Override
        public Function<Pair<Float, IRobot>, ElapsedContainer> getDataProvider() {
            return _elapsed -> new ElapsedContainer(_elapsed.fst);
        }

        public SimpleRotTravel(float _target) {
            target = _target;
        }

        @Override
        public void init(IRobot robot, ElapsedContainer data) {

        }

        @Override
        public void start(IRobot robot, ElapsedContainer data) {

        }

        @Override
        public void loop(IRobot _robot, ElapsedContainer data) {
            Vec2Rot pos = robot.getOdometry().getPos();
            Vec2Rot pow = new Vec2Rot(
                    0,0,
                    Math.signum(pos.x - target) * 0.3f);

            telemetry.addData("power", pow.toString());
            robot.directDrive.directDriveAction().loop(robot,pow);
        }
    }


    LaunchAutoSequenceAction<ElapsedContainer> actionExecutor;
    TimerWrapper timer;

    IAction<Boolean> setTargetToBallState(SpindexerController.BallState state) {
        return IAction.From.loop((r, b)-> {if (b) robot.spindexerCommander.setTargetSlotState(state);});
    }

    IAction<Boolean> moveToTarget(SpindexerController.BallState state) {
        return IAction.From.loop((r, b) -> {if (b) robot.spindexerCommander.goToBallState().loop(r, state);});
    }

    IAction<Boolean> moveToTargetIdx(int idx) {



        return IAction.From.loop((r, b) -> {
            SpindexerController.BallState state;

            if (idx == 0) {
                state = obeliskPattern.first;
            }
            else if (idx == 1) {
                state = obeliskPattern.second;
            }
            else {
                state = obeliskPattern.third;
            }
            if (b) robot.spindexerCommander.goToBallState().loop(r, state);
        });

    }

    ServoWrapper kickerServo;
    ObeliskPattern<SpindexerController.BallState> obeliskPattern;

    AutoActionSequence<ElapsedContainer> getSequence() {
        return new AutoActionSequence<>(
                new WaitAutoAction(3f),
                new IActionAutoAction<>(0.1f, robot.resetHeadingAction, it -> true),
                new SimpleBackwardTravel(-100),
                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new WaitAutoAction(1.5f),
                new SimpleRotTravel(1f),
                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new SimpleRotTravel(0f),
                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),

                //new IActionAutoAction<>(0.1f, robot.outtakeController.stateMachineIdleToggleAction(), it -> true),
                //new IActionAutoAction<>(0.1f, robot.outtakeController.stateMachineIdleToggleAction(), it -> false),
                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new WaitAutoAction(1f),
                new IActionAutoAction<>(1f, moveToTargetIdx(0), it -> true),
                new IActionAutoAction<>(2f, robot.spindexerController.powerMotor(), it -> timer.get()),
                new IActionAutoAction<>(0.4f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.84f)), it -> true),
                new IActionAutoAction<>(0.1f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.31f)), it -> true),
                new IActionAutoAction<>(0.1f, IAction.From.loop((r, t) -> {robot.spindexerCommander.setNearestBallState(robot.spindexerController.getMotorPos(), SpindexerController.BallState.NONE);}), it -> true),
                new WaitAutoAction(0.3f),
                new IActionAutoAction<>(1f, moveToTargetIdx(1), it -> true),
                new IActionAutoAction<>(2f, robot.spindexerController.powerMotor(), it -> timer.get()),
                new IActionAutoAction<>(0.4f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.84f)), it -> true),
                new IActionAutoAction<>(0.1f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.31f)), it -> true),
                new IActionAutoAction<>(0.1f, IAction.From.loop((r, t) -> {robot.spindexerCommander.setNearestBallState(robot.spindexerController.getMotorPos(), SpindexerController.BallState.NONE);}), it -> true),
                new WaitAutoAction(0.3f),
                new IActionAutoAction<>(1f, moveToTargetIdx(2), it -> true),
                new IActionAutoAction<>(2f, robot.spindexerController.powerMotor(), it -> timer.get()),

                new IActionAutoAction<>(0.4f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.84f)), it -> true),
                new IActionAutoAction<>(0.1f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.31f)), it -> true),
                new IActionAutoAction<>(0.1f, IAction.From.loop((r, t) -> {robot.spindexerCommander.setNearestBallState(robot.spindexerController.getMotorPos(), SpindexerController.BallState.NONE);}), it -> true),




                /*new WaitAutoAction(1.5f),
                new IActionAutoAction<>(0.1f, robot.intakeController.motorPowerToggleAction(), it -> true),
                new IActionAutoAction<>(0.1f, robot.resetHeadingAction, it -> true),
                new SimpleBackwardTravel(-0.5f*(float)Math.sqrt(2*24*24)),
                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new WaitAutoAction(1.5f),
                new SimpleRotTravel((float)Math.atan2(3, 6)),
                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new IActionAutoAction<>(0.1f, robot.outtakeController.stateMachineIdleToggleAction(), it -> true),
                new IActionAutoAction<>(0.1f, robot.outtakeController.stateMachineIdleToggleAction(), it -> false),
                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new WaitAutoAction(4f),
                // pulse
                new IActionAutoAction<>(1f, robot.transferController.transferPowerAction(), it -> 1f),
                new IActionAutoAction<>(1.5f, robot.transferController.transferPowerAction(), it -> 0f),
                new IActionAutoAction<>(1f, robot.transferController.transferPowerAction(), it -> 1f),
                new IActionAutoAction<>(1.5f, robot.transferController.transferPowerAction(), it -> 0f),
                new IActionAutoAction<>(1f, robot.transferController.transferPowerAction(), it -> 1f),
                new IActionAutoAction<>(3f, robot.transferController.transferPowerAction(), it -> 0f),
                new IActionAutoAction<>(1f, robot.transferController.transferPowerAction(), it -> 1f),
                new IActionAutoAction<>(1.5f, robot.transferController.transferPowerAction(), it -> 0f),
                new IActionAutoAction<>(1.5f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(-0.0f, 0.5f, 0f)),
                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                */
                new WaitAutoAction(30f)

                /*

                 new WaitAutoAction(1.5f),
                new IActionAutoAction<>(0.1f, robot.intakeController.motorPowerToggleAction(), it -> true),
                new IActionAutoAction<>(0.1f, robot.intakeController.motorPowerToggleAction(), it -> false),
                new IActionAutoAction<>(0.1f, robot.resetHeadingAction, it -> true),
                //new IActionAutoAction<>(1.47f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 1, 0)),
                new SimpleBackwardTravel(3.7f*(float)Math.sqrt(2*24*24)),
                //new LocalizationDisplacement(new Vec2Rot(0, 24 * 5, 0), transControlBuild, rotControlBuild.build()),
                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new WaitAutoAction(1.5f),
                //new IActionAutoAction<>(0.2f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, -1)),
                //new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new IActionAutoAction<>(0.1f, robot.outtakeController.stateMachineIdleToggleAction(), it -> true),
                new IActionAutoAction<>(0.1f, robot.outtakeController.stateMachineIdleToggleAction(), it -> false),
                //new IActionAutoAction<>(1.5f, robot.stateMachineDrive.lookAtAprilTagAction(), it -> {
                //    robot.stateMachineDrive.stateMachineAction().loop(robot, 0);
                //    return true;
                //}),
                //new IActionAutoAction<>(0.4f, robot.stateMachineDrive.lookAtAprilTagAction(), it -> false),
                new WaitAutoAction(4f),
// pulse
                new IActionAutoAction<>(1f, robot.transferController.transferPowerAction(), it -> 1f),
                new IActionAutoAction<>(1.5f, robot.transferController.transferPowerAction(), it -> 0f),
                new IActionAutoAction<>(1f, robot.transferController.transferPowerAction(), it -> 1f),
                new IActionAutoAction<>(1.5f, robot.transferController.transferPowerAction(), it -> 0f),
                new IActionAutoAction<>(1f, robot.transferController.transferPowerAction(), it -> 1f),
                new IActionAutoAction<>(1.5f, robot.transferController.transferPowerAction(), it -> 0f),
                new IActionAutoAction<>(1f, robot.transferController.transferPowerAction(), it -> 1f),
                new IActionAutoAction<>(1.5f, robot.transferController.transferPowerAction(), it -> 0f),
                new IActionAutoAction<>(1f, robot.transferController.transferPowerAction(), it -> 1f),
                new IActionAutoAction<>(1.5f, robot.transferController.transferPowerAction(), it -> 0f),
                new IActionAutoAction<>(1.5f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0.5f, 0, 0f)),
                new WaitAutoAction(30f)

                 */
        );
    }

    ThunderclapRobot robot;

    @Override
    public void init() {
        robot = new ThunderclapRobot(hardwareMap);
        robot.spindexerController = new SpindexerController.SpindexerPositionController(
                new PIDF.Weights(/*50,1.2f,1*/0.03f, 0.0f, 0.000f,0,0f,1),
                Util.also(new DcMotorWrapper(hardwareMap.dcMotor.get("spindexer"), true,
                        MotorSpec.GOBILDA_5203_2402_0019), m -> m.setDirection(IMotor.Direction.FORWARD)),
                robot.spindexerCommander::getTargetAngle
        );

        kickerServo = new ServoWrapper(hardwareMap.servo.get("kicker"));
        actionExecutor = new LaunchAutoSequenceAction<>(getSequence());
        robot.spindexerCommander.setSlots(
                SpindexerController.BallState.PURPLE,
                SpindexerController.BallState.GREEN,
                SpindexerController.BallState.PURPLE
        );


        timer = new TimerWrapper();
    }

    @Override
    public void loop() {
        robot.camera.cameraDetectAction().loop(robot, 0);
        robot.camera.getLastReadObeliskPattern().ifPresent(it -> obeliskPattern = it);
        robot.outtakeController.stateMachineAction().loop(robot, 0);
        robot.directDrive.splitAction().loop(robot, 0);
        robot.outtakeController.controlLoopAction().loop(robot, timer.get());
        robot.outtakeController.stateMachineControlLoopAction().loop(robot, timer.get());
        //robot.stateMachineDrive.controlLoopAction().loop(robot, timer.get());
        robot.intakeController.getTelemetryAction().loop(robot, telemetry);
        robot.getOdometry().loop(timer.get());
        robot.getOdometry().getTelemetryAction().loop(robot, telemetry);
        actionExecutor.loop(robot, true);
        timer.reset();
    }
}
