package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.action.AxisSplitterAction;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.hardware.GamepadWrapper;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpmode;
import org.firstinspires.ftc.teamcode.robot.ClankerHawk2A;

@TeleOp(name = "Outtake Drive Test", group = "Testing")
public class OuttakeDriveTestOpmode extends OpMode {
    TeleOpmode<ClankerHawk2A> opmode;

    @Override
    public void init() {
        // The function of teleop opmodes is to link actions of the robot to the input response manager
        // This opmode relies on the [ClankerHawk2A] robot
        opmode = new TeleOpmode<>(
                new GamepadWrapper(gamepad1), new GamepadWrapper(gamepad2),
                new ClankerHawk2A(hardwareMap),
                telemetry,
                (robot, b)-> b
                        .leftStickAction(
                                robot.directDrive.splitAction().leftSetter()
                        )
                        .rightStickAction(
                                robot.directDrive.splitAction().rightSetter()
                        )
                        .loops(
                                robot.directDrive.splitAction(),
                                robot.flywheelController.setMotorPowerAction(),
                                robot.intakeController.setMotorPowerAction()
                        )
                        .rightTriggerAction(
                                robot.flywheelController.setPowerAction()
                        )
                        .leftTriggerAction(
                                robot.intakeController.setPowerAction()
                        )
                        .AAction(
                                robot.flywheelController.setLockAction()
                        )
                        .BAction(
                                robot.intakeController.setLockAction()
                        )
                        .XAction(
                            robot.resetHeadingAction
                        )
                        .telemetry(
                                robot.getOdometry().getTelemetryAction(),
                                robot.getDrive().getTelemetryAction(),
                                robot.flywheelController.getTelemetryAction(),
                                robot.getIMU().getTelemetryAction()
                        )
                        .DPadAction(
                                AxisSplitterAction.TwoWay(
                                        new EmptyAction<>(),
                                        robot.flywheelController.setHoodPowerAction()
                                )
                        )
                        .timeLoops(
                                robot.getOdometry().getLoopAction()
                        )
                        .build(),
                (r, b) -> b.build());

    }

    @Override
    public void loop() {
        opmode.loop();
    }
}
