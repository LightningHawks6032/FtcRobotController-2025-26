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
import org.firstinspires.ftc.teamcode.control.IControlLoop;
import org.firstinspires.ftc.teamcode.control.IControlLoopBuildOpt;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.Pair;
import org.firstinspires.ftc.teamcode.util.TimerWrapper;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

import java.util.function.Function;

@Autonomous(name="Far Red")
public class FarRedOpmode extends OpMode {
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
    class LocalizationDisplacement implements IAutoAction<ElapsedContainer> {

        final static float TRANS_THRESHOLD = 5f;
        final static float ROT_THRESHOLD = 0.3f;
        Vec2Rot offset, target;

        IControlLoop cx, cy, cr;

        @Override
        public boolean isDone(float duration) {
            Vec2Rot diff = target.componentwiseSub(robot.getOdometry().getPos());
            return diff.asVec2().mag() <= TRANS_THRESHOLD && Math.abs(diff.r) <= ROT_THRESHOLD;
        }

        @Override
        public Function<Pair<Float, IRobot>, ElapsedContainer> getDataProvider() {
            return _elapsed -> new ElapsedContainer(_elapsed.fst);
        }

        public LocalizationDisplacement(Vec2Rot _offset, IControlLoopBuildOpt<? extends IControlLoop> transControl, IControlLoop rotControl) {
            offset = _offset;
            cx = transControl.build();
            cy = transControl.build();
            cr = rotControl;
        }

        @Override
        public void init(IRobot robot, ElapsedContainer data) {
            target = robot.getOdometry().getPos().componentwiseAdd(offset);
        }

        @Override
        public void start(IRobot robot, ElapsedContainer data) {

        }

        @Override
        public void loop(IRobot _robot, ElapsedContainer data) {
            Vec2Rot pos = robot.getOdometry().getPos();
            robot.directDrive.directDriveAction().loop(robot,
                    new Vec2Rot(
                            cx.loop(pos.x, target.x, timer.get()),
                            cy.loop(pos.y, target.y, timer.get()),
                            cr.loop(pos.r, target.r, timer.get())
                    )
            );
        }
    }



    LaunchAutoSequenceAction<ElapsedContainer> actionExecutor;
    TimerWrapper timer;
    IControlLoopBuildOpt<? extends IControlLoop> transControlBuild = new PIDF.BuildOpt(new PIDF.Weights(
        0.0025f,
            0,
            0.0003f,
            0,
            0.01f,1f
    ));
    IControlLoopBuildOpt<? extends IControlLoop> rotControlBuild = new PIDF.BuildOpt(new PIDF.Weights(
            0.8f, 0f,0.05f,0,0.01f,1
    ));
    AutoActionSequence<ElapsedContainer> getSequence() {
        return new AutoActionSequence<>(
                new ActionAutoAction<>(0.1f, robot.resetHeadingAction, it -> true),
                //new ActionAutoAction<>(1.67f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 1, 0)),

                new LocalizationDisplacement(new Vec2Rot(0, 24 * 5, 0), transControlBuild, rotControlBuild.build()),
                new ActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new WaitAutoAction(1.5f),
                new ActionAutoAction<>(0.2f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 1)),
                new ActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), it -> new Vec2Rot(0, 0, 0)),
                new ActionAutoAction<>(0.1f, robot.outtakeController.stateMachineIdleToggleAction(), it -> true),
                new ActionAutoAction<>(0.1f, robot.outtakeController.stateMachineIdleToggleAction(), it -> false),
                new WaitAutoAction(2f),
                new ActionAutoAction<>(5f, robot.transferController.transferPowerAction(), it -> 1f),
                new ActionAutoAction<>(0.1f, robot.transferController.transferPowerAction(), it -> 0f)

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
        actionExecutor.loop(robot, gamepad1.a);
        timer.reset();
    }
}
