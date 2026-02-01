package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.PIDDrive;
import org.firstinspires.ftc.teamcode.components.action.AxisSplitterAction;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.GamepadWrapper;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpmode;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;

@TeleOp()
public class TestPIDDriveOpMode extends OpMode {
    ThunderclapRobot robot;
    TeleOpmode<ThunderclapRobot> opmode;
    PIDDrive drive;

    @Override
    public void init() {
        robot = new ThunderclapRobot(hardwareMap);
        drive = new PIDDrive(() -> robot.getOdometry().getPos().r, robot.directDrive, new PIDF.Weights(
                1f, 0.10f, 0.05f, 0, 0.01f, 1
        ));
        opmode = new TeleOpmode<>(new GamepadWrapper(gamepad1), new GamepadWrapper(gamepad2),
                robot, telemetry,
                (r, b) -> b
                        .leftStickAction(
                                drive.setXY
                        )
                        .rightStickAction(
                                AxisSplitterAction.TwoWay(
                                        drive.setR,
                                        new EmptyAction<>()
                                )
                        )
                        .timeLoops(
                                drive.driveLoop,
                                robot.getOdometry().getLoopAction()
                        )
                        .telemetry(
                                robot.getOdometry(),
                                drive
                        )
                        .build(),
                TeleOpmode.EmptyGamepad());
    }

    @Override
    public void loop() {
        opmode.loop();
    }
}
