package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.action.AxisSplitterAction;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
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
                                /*.AAction(
                                        robot.stateMachineDrive.lookAtAprilTagAction()
                                )
                                .BAction(
                                        robot.stateMachineDrive.lookAtAprilTagActionFixed()
                                )*/
                                .XAction(
                                        robot.resetHeadingAction
                                )
                                /*.DPadAction(
                                        robot.stateMachineDrive.lookAtUserDirectionAction()
                                )*/
                                .telemetry(
                                        robot.getOdometry(),
                                        robot.getDrive(),
                                        robot.getIMU()
                                        //robot.stateMachineDrive
                                )
                                /*.loops(
                                        //robot.directDrive.splitAction()
                                        //robot.stateMachineDrive.stateMachineAction()
                                )*/
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
                                .leftBumperAction(
                                        robot.outtakeController.stateMachineIdleToggleAction()
                                )
                                .rightBumperAction(
                                        robot.outtakeController.stateMachineControlSpeedToggleAction()
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
                                        robot.camera,
                                        robot.outtakeController.stateMachineStateTelemetry()
                                )
                                .timeLoops(
                                        robot.outtakeController.controlLoopAction(),
                                        robot.outtakeController.stateMachineControlLoopAction(),
                                        robot.stateMachineDrive.controlLoopAction()
                                )
                                .loops(
                                        robot.camera.cameraDetectAction(),
                                        robot.outtakeController.stateMachineAction(),
                                        robot.hoodController.setHoodPositionDistanceAction()
                                )
                                .build()
        );

        hardwareMap.servo.get("hood").setPosition(0.8);
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
