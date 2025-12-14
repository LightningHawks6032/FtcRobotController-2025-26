package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.IControlLoop;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpmode;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.TimerWrapper;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;

@TeleOp
public class LookAtAngleOpmode extends OpMode {

    ThunderclapRobot robot;
    TimerWrapper timer;
    PIDF.Controller control;

    float desiredAngle = 0f;
    float pow = 0f;

    @Override
    public void init() {
        robot = new ThunderclapRobot(hardwareMap);
        timer = new TimerWrapper();
        control = new PIDF.Controller(
                new PIDF.Weights(
                        1f, 0.1f,0.05f,0f,0.01f,1

                )
        );
    }



    @Override
    public void loop() {
        robot.getOdometry().loop(timer.get());

        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
            desiredAngle = /*180 / (float)Math.PI */ Util.normAngle2Pi((float)Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
        }
        else {desiredAngle = 0f;}

        pow = control.loop(
                /*180 / (float)Math.PI * */Util.computeShortestSignedAngle(Util.normAngle2Pi(robot.getOdometry().getPos().r), desiredAngle),
                0f,
                timer.get()
        );

        robot.directDrive.directDriveAction().loop(robot, new Vec2Rot(0, 0,pow
                ));

        telemetry.addData("robot angle", Util.normAngle(robot.getOdometry().getPos().r));
        telemetry.addData("desiredAngle", desiredAngle);
        telemetry.addData("power", pow);
        telemetry.addData("Distance", Util.computeShortestSignedAngle(Util.normAngle2Pi(robot.getOdometry().getPos().r), desiredAngle));

        timer.reset();
    }
}
