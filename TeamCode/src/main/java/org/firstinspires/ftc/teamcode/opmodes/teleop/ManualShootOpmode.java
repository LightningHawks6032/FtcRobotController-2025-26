package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.components.action.AxisSplitterAction;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;
import org.firstinspires.ftc.teamcode.hardware.ServoWrapper;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpmode;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.SpindexerController;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.ButtonCounter;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(name="Manual Shoot", group="Comp")
public class ManualShootOpmode extends OpMode {

    TeleOpmode<ThunderclapRobot> opmode;

    Toggle spindexerSlots;
    ButtonCounter leftSpindexer, rightSpindexer;

    int spindexerIdx;
    @Override
    public void init() {
        spindexerIdx = 0;
        leftSpindexer = new ButtonCounter();
        rightSpindexer = new ButtonCounter();
        spindexerSlots = new Toggle();
        opmode = new TeleOpmode<>(this, new ThunderclapRobot(hardwareMap),
                (robot, input) ->
                        input
                                .leftStickAction(
                                        robot.directDrive.splitAction().leftSetter()
                                )
                                .rightStickAction(
                                        robot.directDrive.splitAction().rightSetter()
                                )
                                .leftTriggerAction(
                                        IAction.From.loop((r, f) -> robot.directDrive.slowModeAction().loop(r, f >= 0.6f))
                                )
                                .rightTriggerAction(
                                        IAction.From.loop((r, f) -> robot.kickerController.getKickerAction().loop(r, f >= 0.85f))

                                )
                                .rightBumperAction(
                                        robot.directDrive.fastModeAction()
                                )
                                /*.AAction(
                                        robot.kickerController.getKickerAction(),
                                        IAction.From.loop((r, b) -> {if (b) robot.spindexerCommander.setNearestBallState(robot.spindexerController.getMotorPos(), SpindexerController.BallState.NONE);})
                                )*//*
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
                                .loops(
                                        robot.directDrive.splitAction()
                                        //robot.stateMachineDrive.stateMachineAction()
                                )
                                .timeLoops(
                                        robot.getOdometry().getLoopAction(),
                                        robot.spindexerController.powerMotor()
                                )
                                .build(),
                (robot, input) ->
                        input
                                .AAction(
                                        robot.intakeController.ejectPowerAction()
                                )
                                .BAction(
                                        IAction.From.loop(
                                                (_r, b) -> {
                                                    if (b) {
                                                        robot.spindexerCommander.setNearestBallState(robot.spindexerController.getMotorPos(), SpindexerController.BallState.PURPLE);
                                                    }
                                                }
                                        )
                                )
                                /*.YAction(
                                        IAction.From.loop((r, b) -> {
                                            kicker.loop(b);
                                            kickerServo.setPosition(kicker.toggle() ? )
                                        })
                                )*/
                                .XAction(
                                        robot.intakeController.motorPowerToggleAction()
                                )
                                .rightBumperAction(
                                        robot.intakeSwingController.getSwingIntakeAction()//robot.intakeAutomationController.intakeModeAction()
                                )
                                .rightStickAction(
                                        AxisSplitterAction.TwoWay(
                                                new EmptyAction<>(),
                                                robot.outtakeController.setMotorSpeedAction()
                                        )
                                )
                                .DPadAction(
                                        AxisSplitterAction.FourWay(
                                                new EmptyAction<>(),
                                                IAction.From.loop((r, b) -> {
                                                    //spindexerSlots.loop(b);
                                                    //robot.spindexerCommander.goToBallState().loop(r, spindexerSlots.toggle() && !robot.kickerController.isRunning() ? SpindexerController.BallState.PURPLE : SpindexerController.BallState.NONE);
                                                    robot.rightSpindexer.loop(b);

                                                }),
                                                new EmptyAction<>(),
                                                IAction.From.loop((r, b) -> {
                                                    //spindexerSlots.loop(b);
                                                    //robot.spindexerCommander.goToBallState().loop(r, spindexerSlots.toggle() && !robot.kickerController.isRunning() ? SpindexerController.BallState.PURPLE : SpindexerController.BallState.NONE);
                                                    robot.leftSpindexer.loop(b);

                                                })
                                        )
                                )
                                .telemetry(
                                        robot.intakeController,
                                        robot.outtakeController,
                                        robot.outtakeController.stateMachineTelemetry(),
                                        robot.hoodController,
                                        robot.camera,
                                        robot.outtakeController.stateMachineStateTelemetry(),
                                        robot.colorSensor,
                                        robot.spindexerCommander,
                                        robot.spindexerController,
                                        robot.ballStateDeterminer
                                )
                                .timeLoops(
                                        robot.outtakeController.controlLoopAction(),
                                        robot.outtakeController.stateMachineControlLoopAction(),
                                        robot.stateMachineDrive.controlLoopAction(),
                                        robot.spindexerController.powerMotor()

                                )
                                .loops(
                                        robot.camera.cameraDetectAction(),
                                        robot.outtakeController.stateMachineAction(),
                                        robot.hoodController.setHoodPositionDistanceAction(),
                                        IAction.From.loop((r1, o) -> {
                                            if (Math.abs(robot.colorSensor.getDistance() - 1.3f) <= 0.3f) robot.spindexerCommander.setNearestBallStateGivenBallWithinThresh(robot.spindexerController.getMotorPos(), SpindexerController.BallState.PURPLE, 4);
                                            if (robot.intakeController.getMotorCurrent() >= 5) {
                                                robot.intakeSwingController.getSwingIntakeAction().loop(robot, true);
                                                robot.intakeSwingController.getSwingIntakeAction().loop(robot, false);

                                                robot.intakeController.motorPowerToggleAction().loop(robot, true);
                                                robot.intakeController.motorPowerToggleAction().loop(robot, false);

                                            }
                                        })
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
