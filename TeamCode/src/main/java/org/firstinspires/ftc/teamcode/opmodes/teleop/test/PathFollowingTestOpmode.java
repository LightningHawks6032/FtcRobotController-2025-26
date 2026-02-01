package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.TimerWrapper;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

@TeleOp(name="Path following test opmode")
public class PathFollowingTestOpmode extends OpMode {

    ThunderclapRobot robot;

    TimerWrapper timer;
    float path_t = 0f;
    final float lookahead = 0.2f;
    final float maxSpeed = 3f;

    PIDF.Controller posX, posY, velX, velY;

    Vec2 prevPos;
    Vec2Rot getPathInfo(float p_t) {
        p_t = 0.7f * p_t;
        float tx = 2 * (float)Math.sin(p_t / 2.0);
        float ty = 2 * (float)Math.sin(p_t);
        float dx = 1 * (float)Math.cos(p_t / 2.0);
        float dy = 2 * (float)Math.cos(p_t);
        float path_speed = (float)Math.sqrt(dx*dx + dy*dy);

        return new Vec2Rot(tx, ty, path_speed);
    }

    @Override
    public void init() {

        robot = new ThunderclapRobot(hardwareMap);
        timer = new TimerWrapper();

        PIDF.BuildOpt posBuildOpt = new PIDF.BuildOpt(new PIDF.Weights(
                3,0,0,0,0.01f,1
        ));
        PIDF.BuildOpt velBuildOpt = new PIDF.BuildOpt(new PIDF.Weights(
                5,0,0,0,0.01f,1
        ));

        posX = posBuildOpt.build(); posY = posBuildOpt.build();
        velX = velBuildOpt.build(); velY = velBuildOpt.build();
        prevPos = robot.getOdometry().getPos().asVec2().scale(1/100f);
    }

    @Override
    public void loop() {
        float dt = timer.get();
        robot.getOdometry().loop(dt);
        timer.reset();

        Vec2 robotPos = robot.getOdometry().getPos().asVec2().scale(1/100f);
        Vec2 robotVel = robot.getOdometry().getVel().asVec2().scale(1/100f);
        telemetry.addData("pos", robotPos.toString());
        telemetry.addData("vel mag", robotVel.mag());
        Vec2Rot nowPos = getPathInfo(path_t);
        Vec2Rot lookaheadPos = getPathInfo(path_t + lookahead / Math.max(nowPos.r, 0.1f));
        telemetry.addData("target pos", nowPos.toString());
        Vec2 velCmd = new Vec2(
                posX.loop(robotPos.x, lookaheadPos.x, dt),
                posY.loop(robotPos.y, lookaheadPos.y, dt)
        );

        Vec2 accCmd = new Vec2(
                velX.loop(robotVel.x, Util.clamp(velCmd.x, -maxSpeed, maxSpeed), dt),
                velY.loop(robotVel.y, Util.clamp(velCmd.y, -maxSpeed, maxSpeed), dt)
        );

        telemetry.addData("cmd", accCmd.toString());

        robot.directDrive.directDriveAction().loop(robot, accCmd.asVec2Rot());

        float dist = robotPos.sub(prevPos).mag();
        path_t += dist / Math.max(nowPos.r, 0.01f);
        prevPos = robotPos;
        telemetry.addData("path_t", path_t);
    }
}
