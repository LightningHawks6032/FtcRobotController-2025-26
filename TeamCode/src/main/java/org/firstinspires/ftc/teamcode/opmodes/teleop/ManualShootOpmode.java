package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.action.AxisSplitterAction;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpmode;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;

@TeleOp(name="Manual Shoot", group="Comp")
public class ManualShootOpmode extends OpMode {

    TeleOpmode<ThunderclapRobot> opmode;

    @Override
    public void init() {
        opmode = new TeleOpmode<>(this, new ThunderclapRobot(hardwareMap),
                (robot, input) ->
                        input
                                .leftStickAction(
                                        robot.directDrive.splitAction().leftSetter()
                                )
                                .rightStickAction(
                                        robot.directDrive.splitAction().rightSetter()
                                )
                                .leftBumperAction(
                                        robot.directDrive.slowModeAction()
                                )
                                .rightBumperAction(
                                        robot.directDrive.fastModeAction()
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
                                        robot.directDrive.splitAction()
                                )
                                .timeLoops(
                                        robot.getOdometry().getLoopAction()
                                )
                                .build(),
                (robot, input) ->
                        input
                                .AAction(
                                        robot.intakeController.ejectPowerAction()
                                )
                                .BAction(
                                        robot.outtakeController.lockToggleAction()
                                )
                                .XAction(
                                        robot.intakeController.motorPowerToggleAction()
                                )
                                /*.YAction(
                                        robot.outtakeController.setMotorSpeedButtonAction()
                                )*/
                                .leftBumperAction(
                                        robot.outtakeController.stateMachineIdleToggleAction()
                                )
                                .rightBumperAction(
                                        robot.outtakeController.stateMachineControlSpeedToggleAction()
                                )
                                .leftStickAction(
                                        AxisSplitterAction.TwoWay(
                                                new EmptyAction<>(),
                                                robot.hoodController.setHoodPositionAction()
                                        )
                                )
                                .rightStickAction(
                                        AxisSplitterAction.TwoWay(
                                                new EmptyAction<>(),
                                                robot.outtakeController.setMotorSpeedAction()
                                        )
                                )
                                .DPadAction(
                                        AxisSplitterAction.TwoWay(
                                                new EmptyAction<>(),
                                                robot.transferController.transferPowerAction()
                                        )
                                )
                                .telemetry(
                                        robot.intakeController,
                                        robot.outtakeController,
                                        robot.outtakeController.stateMachineTelemetry(),
                                        robot.hoodController,
                                        robot.camera
                                )
                                .timeLoops(
                                        robot.outtakeController.controlLoopAction()
                                )
                                .loops(
                                        robot.camera.cameraDetectAction(),
                                        robot.outtakeController.stateMachineAction()
                                )
                                .build()
        );
    }

    @Override
    public void loop() {
        opmode.loop();
    }

    @Override
    public void stop() {
        hardwareMap.servo.get("hood").setPosition(0.4);
    }
}
