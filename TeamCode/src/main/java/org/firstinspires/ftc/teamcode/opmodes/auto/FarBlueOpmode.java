package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.WaitAutoAction;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.Pair;
import org.firstinspires.ftc.teamcode.util.TimerWrapper;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

import java.util.function.Function;

@Autonomous(name="Far Blue")
public class FarBlueOpmode extends OpMode {
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

    LaunchAutoSequenceAction<ElapsedContainer> actionExecutor;
    TimerWrapper timer;
    AutoActionSequence<ElapsedContainer> getSequence() {
        return new AutoActionSequence<>(
                new WaitAutoAction(1.5f),
                new FarRedOpmode.ActionAutoAction<>(0.1f, robot.intakeController.motorPowerToggleAction(), it -> true),
                new FarRedOpmode.ActionAutoAction<>(0.1f, robot.intakeController.motorPowerToggleAction(), it -> false),
                new FarRedOpmode.ActionAutoAction<>(0.1f, robot.resetHeadingAction, it -> true),
                new FarRedOpmode.ActionAutoAction<>(1.47f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 1, 0)),

                //new LocalizationDisplacement(new Vec2Rot(0, 24 * 5, 0), transControlBuild, rotControlBuild.build()),
                new FarRedOpmode.ActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new WaitAutoAction(1.5f),
                new FarRedOpmode.ActionAutoAction<>(0.2f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, -1)),
                new FarRedOpmode.ActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new FarRedOpmode.ActionAutoAction<>(0.1f, robot.outtakeController.stateMachineIdleToggleAction(), it -> true),
                new FarRedOpmode.ActionAutoAction<>(0.1f, robot.outtakeController.stateMachineIdleToggleAction(), it -> false),
                new WaitAutoAction(2f),
                new FarRedOpmode.ActionAutoAction<>(5f, robot.transferController.transferPowerAction(), it -> 1f),
                new FarRedOpmode.ActionAutoAction<>(0.1f, robot.transferController.transferPowerAction(), it -> 0f),
                new FarRedOpmode.ActionAutoAction<>(1.5f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(-0.5f, -0.5f, 0f)),
                new FarRedOpmode.ActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0))
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
        actionExecutor.loop(robot, true);
        timer.reset();
    }
}
