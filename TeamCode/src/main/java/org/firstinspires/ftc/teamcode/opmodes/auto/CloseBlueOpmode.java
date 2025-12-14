package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IActionAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.IAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.WaitAutoAction;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.Pair;
import org.firstinspires.ftc.teamcode.util.TimerWrapper;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

import java.util.function.Function;

@Autonomous(name="Close Blue")
public class CloseBlueOpmode extends OpMode {
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
                    Math.signum(pos.x - target)
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

    public static class AlternateIntake implements IAutoAction<ElapsedContainer> {

        ThunderclapRobot robot;
        float duration;
        float rest;
        float on;

        @Override
        public boolean isDone(float _duration) {
            return _duration >= duration;
        }

        @Override
        public Function<Pair<Float, IRobot>, ElapsedContainer> getDataProvider() {
            return p -> new ElapsedContainer(p.fst);
        }

        public AlternateIntake(float _duration, int _numCycle, float _on) {
            duration = _duration;
            on = _on;
            rest = duration / _numCycle - on;
        }

        @Override
        public void init(IRobot robot, ElapsedContainer data) {

        }

        @Override
        public void start(IRobot robot, ElapsedContainer data) {

        }

        @Override
        public void loop(IRobot robot, ElapsedContainer data) {

        }
    }

    LaunchAutoSequenceAction<ElapsedContainer> actionExecutor;
    TimerWrapper timer;
    AutoActionSequence<ElapsedContainer> getSequence() {
        return new AutoActionSequence<>(
                new WaitAutoAction(1.5f),
                new IActionAutoAction<>(0.1f, robot.intakeController.motorPowerToggleAction(), it -> true),
                //new IActionAutoAction<>(0.1f, robot.intakeController.motorPowerToggleAction(), it -> false),
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

                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new WaitAutoAction(30f)
        );
    }

    ThunderclapRobot robot;

    @Override
    public void init() {
        robot = new ThunderclapRobot(hardwareMap);

        actionExecutor = new LaunchAutoSequenceAction<>(getSequence());

        timer = new TimerWrapper();
    }

    @Override
    public void loop() {
        robot.camera.cameraDetectAction().loop(robot, 0);
        robot.outtakeController.stateMachineAction().loop(robot, 0);
        robot.outtakeController.controlLoopAction().loop(robot, timer.get());
        robot.outtakeController.stateMachineControlLoopAction().loop(robot, timer.get());
        robot.hoodController.setHoodPositionDistanceAction().loop(robot, 0);
        robot.getOdometry().loop(timer.get());
        robot.stateMachineDrive.controlLoopAction().loop(robot, timer.get());
        actionExecutor.loop(robot, true);
        timer.reset();
    }
}

