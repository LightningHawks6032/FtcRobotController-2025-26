package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.DirectDrive;
import org.firstinspires.ftc.teamcode.components.action.PredicateAction;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.drive.BezierPath;
import org.firstinspires.ftc.teamcode.hardware.drive.IPath;
import org.firstinspires.ftc.teamcode.hardware.drive.PathPoint;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpmode;
import org.firstinspires.ftc.teamcode.robot.ClankerHawk2A;
import org.firstinspires.ftc.teamcode.util.TimerWrapper;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;
import java.util.function.BiFunction;

@Autonomous
public class PathFollowingOpmode extends OpMode {


    Queue<IPath<Vec2>> paths;
    TimerWrapper pathTimer, loopTimer;
    PIDF.Controller pidX, pidY;
    ClankerHawk2A robot;
    DirectDrive driveAction;
    Vec2 fixed = new Vec2(150, 150);

    /// we'll fix this for real auto; this is just testing :3c
    TeleOpmode<ClankerHawk2A> teleop;
    @Override
    public void init() {
        robot = new ClankerHawk2A(hardwareMap);
        pathTimer = new TimerWrapper();
        loopTimer = new TimerWrapper();

        pidX = new PIDF.Controller(
                new PIDF.Weights(0.0035f, 0f, 0.1f, 1, 0.3f, 0.1f)
        );
        pidY = new PIDF.Controller(
                new PIDF.Weights(0.0035f, 0f, 0.1f, 1, 0.3f, 0.1f)
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



        driveAction = new DirectDrive(robot.getDrive(),
                ((BiFunction<Vec2, Vec2, Vec2Rot>)(v1, v2) -> {
            return new Vec2Rot(
                    pidX.loop(robot.getOdometry().getPos().x, fixed.x, loopTimer.get()),
                    pidY.loop(robot.getOdometry().getPos().y, fixed.y, loopTimer.get())
            ,0);}).andThen(DirectDrive.fieldCentricFromIMU(robot.getIMU()))

                );

        teleop = new TeleOpmode<>(
                this, robot,
                (robot, b) -> b
                        .leftStickAction(
                                driveAction.splitAction().leftSetter(),
                                robot.directDrive.splitAction().leftSetter()
                        )
                        .rightStickAction(
                                driveAction.splitAction().rightSetter(),
                                robot.directDrive.splitAction().rightSetter()
                        )
                        .XAction(
                                robot.resetHeadingAction
                        )
                        .telemetry(
                                robot.getOdometry(),
                                robot.getDrive(),
                                robot.getIMU()
                        )
                        .loops(
                                new PredicateAction<>(
                                        driveAction.splitAction(),
                                        robot.directDrive.splitAction(),
                                        (r, o) -> gamepad1.right_stick_button
                                )
                        )
                        .timeLoops(
                                robot.getOdometry().getLoopAction()
                        )
                        .build(),
                TeleOpmode.EmptyGamepad()
        );
    }

    PathPoint<Vec2> getPathPoint() {
        return paths.peek().getPoint(pathTimer.get());
    }

    Vec2 getDir(float dt) {
        PathPoint<Vec2> point = getPathPoint();

        Vec2 pos = point.pos.sub(new Vec2(49, 45));

        return new Vec2(
                pidX.loop(robot.getOdometry().getPos().x, pos.x, dt),
                pidY.loop(robot.getOdometry().getPos().y, pos.y, dt)
        ).norm();
    }

    @Override
    public void loop() {
//        robot.getOdometry().getLoopAction().loop(robot, loopTimer.get());
//        robot.getOdometry().getTelemetryAction().loop(robot, telemetry);
//        telemetry.addData("Target", getPathPoint().pos.toString());
//
//        Vec2 dir = getDir(loopTimer.get());
//        telemetry.addData("Dir", dir.toString());
//        driveAction.directDriveAction().loop(robot, new Vec2Rot(dir.scale(0.3f), 0));
//
//        loopTimer.reset();
//        if (pathTimer.get() >= paths.peek().getDuration()) {
//            paths.poll();
//            pathTimer.reset();
//        }
        teleop.loop();
        loopTimer.reset();
    }
}
