package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IActionAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.IAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.WaitAutoAction;
import org.firstinspires.ftc.teamcode.components.ObeliskPattern;
import org.firstinspires.ftc.teamcode.components.action.AxisSplitterAction;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.GamepadWrapper;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;
import org.firstinspires.ftc.teamcode.hardware.ServoWrapper;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpmode;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.SpindexerController;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.Toggle;

import java.util.Optional;

@TeleOp(name="Spindexer Test")
public class SpindexerTestOpmode extends OpMode {
    TeleOpmode<ThunderclapRobot> opmode;
    ThunderclapRobot robot;
    SpindexerController.TargetPositionCommander spindexerCommander;
    SpindexerController.SpindexerPositionController spindexerController;
    SpindexerController.BallStateDeterminer ballStateDeterminer;
    DcMotorWrapper motor;
    ColorSensorWrapper colorSensor;

    Toggle kicker;
    ServoWrapper kickerServo;

    Toggle intakeToggle;
    ServoWrapper intakeServo;

    LaunchAutoSequenceAction<ElapsedContainer> kickerAction;

    IAction<Boolean> setTargetToBallState(SpindexerController.BallState state) {
        return IAction.From.loop((r, b)-> {if (b) spindexerCommander.setTargetSlotState(state);});
    }

    IAction<Boolean> moveToTarget(SpindexerController.BallState state) {
        return IAction.From.loop((r, b) -> {if (b) spindexerCommander.goToBallState().loop(r, state);});
    }

    LaunchAutoSequenceAction<ElapsedContainer> shootObeliskPatternAction(ObeliskPattern<SpindexerController.BallState> obeliskPattern) {
        return new LaunchAutoSequenceAction<>(
                new AutoActionSequence<>(
                        new IActionAutoAction<>(1f, moveToTarget(obeliskPattern.first), it -> true),
                        new IActionAutoAction<>(0.8f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.7f)), it -> true),
                        new IActionAutoAction<>(0.1f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.31f)), it -> true),
                        new WaitAutoAction(0.3f),
                        new IActionAutoAction<>(1f, moveToTarget(obeliskPattern.second), it -> true),
                        new IActionAutoAction<>(0.8f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.7f)), it -> true),
                        new IActionAutoAction<>(0.1f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.31f)), it -> true),
                        new WaitAutoAction(0.3f),
                        new IActionAutoAction<>(1f, moveToTarget(obeliskPattern.third), it -> true),
                        new IActionAutoAction<>(0.8f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.7f)), it -> true),
                        new IActionAutoAction<>(0.1f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.31f)), it -> true)
                )
        );
    }

    LaunchAutoSequenceAction<ElapsedContainer> lastLauncher;
    ObeliskPattern<SpindexerController.BallState> lastObelisk;

    @Override
    public void init() {
        robot = new ThunderclapRobot(hardwareMap);
        motor = new DcMotorWrapper(hardwareMap.dcMotor.get("spindexer"), true,
                MotorSpec.GOBILDA_5203_2402_0019);
        motor.setDirection(IMotor.Direction.FORWARD);
        spindexerCommander = new SpindexerController.TargetPositionCommander();
        spindexerController = new SpindexerController.SpindexerPositionController(
                new PIDF.Weights(/*50,1.2f,1*/0.03f, 0.0f, 0.000f,0,0f,1),
                motor,
                spindexerCommander::getTargetAngle
        );
        colorSensor = new ColorSensorWrapper(hardwareMap.get(NormalizedColorSensor.class, "color sensor"));
        colorSensor.setGain(10);

        ballStateDeterminer = new SpindexerController.BallStateDeterminer(colorSensor::getColor);

        kicker = new Toggle();
        kickerServo = new ServoWrapper(hardwareMap.servo.get("kicker"));


        kickerAction = new LaunchAutoSequenceAction<>(
                new AutoActionSequence<>(
                        new IActionAutoAction<>(0.8f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.7f)), it -> true),
                        new IActionAutoAction<>(0.1f, IAction.From.loop((robot, t) -> kickerServo.setPosition(0.31f)), it -> true),
                        new IActionAutoAction<>(0.1f, IAction.From.loop((robot, t) -> spindexerCommander.setNearestBallState(spindexerController.getMotorPos(), SpindexerController.BallState.NONE)), it -> true)
                        )
        );

        intakeToggle = new Toggle();
        intakeServo = new ServoWrapper(hardwareMap.servo.get("intake servo"));

        opmode = new TeleOpmode<>(
                new GamepadWrapper(gamepad1), new GamepadWrapper(gamepad2),
                robot, telemetry,
                (r, b) -> b
                        .AAction(
                                setTargetToBallState(SpindexerController.BallState.GREEN)
                        )
                        .BAction(
                                setTargetToBallState(SpindexerController.BallState.NONE)
                        )
                        .XAction(
                                setTargetToBallState(SpindexerController.BallState.PURPLE)
                        )
                        .YAction(
                                /*IAction.From.loop((robot1, bool) -> {
                                    Optional<ObeliskPattern<SpindexerController.BallState>> obelisk = robot.camera.getLastReadObeliskPattern();
                                    if (obelisk.isPresent() && lastObelisk != obelisk.get()) {
                                        lastObelisk = obelisk.get();
                                        lastLauncher = shootObeliskPatternAction(lastObelisk);
                                    }
                                    if (lastLauncher != null) {
                                        lastLauncher.loop(r, bool);
                                    }
                                })*/
                                robot.intakeController.motorPowerToggleAction()
                        )
                        .DPadAction(
                                AxisSplitterAction.FourWay(
                                        IAction.From.loop((robot, bool) -> {
                                            intakeToggle.loop(bool);
                                            intakeServo.setPosition(intakeToggle.toggle() ? 1 : 0.2f);
                                        }),
                                        moveToTarget(SpindexerController.BallState.NONE),
                                        moveToTarget(SpindexerController.BallState.GREEN),
                                        moveToTarget(SpindexerController.BallState.PURPLE)

                                )
                        )
                        .rightBumperAction(
                                /*IAction.From.loop((robot, bool) -> {
                                    kicker.loop(bool);
                                    kickerServo.setPosition(kicker.toggle() ? 0.7f : 0.4f);
                                })*/
                                kickerAction
                        )
                        .leftBumperAction(
                                robot.outtakeController.stateMachineIdleToggleAction()
                        )
                        .timeLoops(
                                spindexerController.powerMotor(),
                                robot.outtakeController.controlLoopAction(),
                                robot.outtakeController.stateMachineControlLoopAction()
                        )
                        .loops(
                                robot.outtakeController.stateMachineAction(),
                                robot.camera.cameraDetectAction(),
                                IAction.From.loop((robot, o) -> {
                                    spindexerCommander.setNearestBallStateGivenBallWithinThresh(spindexerController.getMotorPos(), ballStateDeterminer.getPredictedState(), 5);
                                })
                        )
                        .telemetry(
                                spindexerCommander,
                                spindexerController,
                                colorSensor,
                                robot.camera,
                                ballStateDeterminer,
                                robot.intakeController
                        )
                        .build(),
                TeleOpmode.EmptyGamepad()
        );

    }

    @Override
    public void loop() {
        opmode.loop();

    }
}
