package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IActionAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.IAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.WaitAutoAction;
import org.firstinspires.ftc.teamcode.components.DirectDrive;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.components.action.PredicateAction;
import org.firstinspires.ftc.teamcode.control.IControlLoop;
import org.firstinspires.ftc.teamcode.control.IControlLoopBuildOpt;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.drive.BezierPath;
import org.firstinspires.ftc.teamcode.hardware.drive.IPath;
import org.firstinspires.ftc.teamcode.hardware.drive.PathPoint;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpmode;
import org.firstinspires.ftc.teamcode.robot.ClankerHawk2A.ClankerHawk2A;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.Pair;
import org.firstinspires.ftc.teamcode.util.TimerWrapper;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;
import java.util.function.BiFunction;
import java.util.function.Function;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Autonomous
public class PathFollowingOpmode extends OpMode {
    Queue<IPath<Vec2>> paths;
    TimerWrapper pathTimer, loopTimer;
    PIDF.Controller pidX, pidY;
    ThunderclapRobot robot;
    DirectDrive driveAction;
    Vec2 fixed = new Vec2(0, 2 * (float)Math.sqrt(2 * 24 * 24));

    LaunchAutoSequenceAction<ElapsedContainer> actionExecutor;

    class LocalizationTravel implements IAutoAction<ElapsedContainer> {

        final static float TRANS_THRESHOLD = 20f;
        Vec2 target;

        @Override
        public boolean isDone(float duration) {
            Vec2 diff = target.sub(robot.getOdometry().getPos().asVec2());
            return diff.mag() <= TRANS_THRESHOLD;
        }

        @Override
        public Function<Pair<Float, IRobot>, ElapsedContainer> getDataProvider() {
            return _elapsed -> new ElapsedContainer(_elapsed.fst);
        }

        public LocalizationTravel(Vec2 _target) {
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
                    pidX.loop(pos.x, target.x, loopTimer.get()),
                    pidY.loop(pos.y, target.y, loopTimer.get())
            ).norm().scale(0.3f)/*.rotateOrigin((float)robot.getIMU().getAngles().getYaw())*/, 0);

            telemetry.addData("power", pow.toString());
            robot.directDrive.directDriveAction().loop(robot,pow);
        }
    }

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


    AutoActionSequence<ElapsedContainer> getSequence() {
        return new AutoActionSequence<>(
                // add localization displacement
                new WaitAutoAction(1f),
                new IActionAutoAction<>(0.1f, robot.resetHeadingAction, t -> true),
                new SimpleBackwardTravel(4*(float)Math.sqrt(2*24*24)),
                new IActionAutoAction<>(0.1f, robot.directDrive.directDriveAction(), (t) -> new Vec2Rot(0,0,0))
        );
    }

    @Override
    public void init() {
        robot = new ThunderclapRobot(hardwareMap);
        pathTimer = new TimerWrapper();
        loopTimer = new TimerWrapper();

        pidX = new PIDF.Controller(
                new PIDF.Weights(0.5f, 0.1f, 0.05f, 0, 0.1f, 1f)
        );
        pidY = new PIDF.Controller(
                new PIDF.Weights(0.5f, 0.1f, 0.05f, 0, 0.1f, 1f)
        );

        paths = new LinkedList<>(Arrays.asList(
                new BezierPath.Cubic(
                        new Vec2(49, 45),
                        new Vec2(264,24),
                        new Vec2(312, 152),
                        new Vec2(253, 250.5f),
                        60f
                )
        ));

        actionExecutor = new LaunchAutoSequenceAction<>(getSequence());
    }

    PathPoint<Vec2> getPathPoint() {
        return paths.peek().getPoint(pathTimer.get());
    }

    @Override
    public void loop() {
        robot.getOdometry().loop(loopTimer.get());

        robot.getOdometry().getTelemetryAction().loop(robot, telemetry);

        actionExecutor.loop(robot, gamepad1.a);
        loopTimer.reset();
        telemetry.addData("IMU yaw", robot.getIMU().getAngles().getYaw());
        telemetry.addData("Odo yaw", robot.getOdometry().getPos().r);
        telemetry.addData("dist", robot.getOdometry().getPos().asVec2().sub(fixed).mag());
    }
}
