package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IAutoAction;
import org.firstinspires.ftc.teamcode.auto.action.WaitAutoAction;
import org.firstinspires.ftc.teamcode.components.RobotController;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.hardware.GamepadWrapper;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.InputResponseManager;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.util.Pair;

import java.util.function.Function;

@TeleOp(name = "Drive Motor Test", group = "Testing")
public class DriveMotorTestOpmode extends OpMode {

    static class MotorTestAction implements IAutoAction<ElapsedContainer> {

        final float duration;
        final IMotor motor;

        public MotorTestAction(IMotor _motor, float _duration) {
            duration = _duration;
            motor = _motor;
        }

        @Override
        public boolean isDone(float duration) {
            boolean done = duration >= this.duration;
            if (done) {
                motor.setPower(0); // TODO: Make an on stop event
            }
            return done;
        }

        @Override
        public Function<Pair<Float, RobotController>, ElapsedContainer> getDataProvider() {
            return _elapsed -> new ElapsedContainer(_elapsed.fst);
        }

        @Override
        public void init(RobotController robot, ElapsedContainer data) {

        }

        @Override
        public void start(RobotController robot, ElapsedContainer data) {

        }

        @Override
        public void loop(RobotController robot, ElapsedContainer data) {
            motor.setPower(1f);
        }
    }

    static AutoActionSequence<ElapsedContainer> testDrive(DriveMotors drive) {
        return new AutoActionSequence<>(
            new MotorTestAction(drive.ul(), 1f),
            new WaitAutoAction(1f),
            new MotorTestAction(drive.ur(), 1f),
            new WaitAutoAction(1f),
            new MotorTestAction(drive.dr(), 1f),
            new WaitAutoAction(1f),
            new MotorTestAction(drive.dl(), 1f)
        );
    }


    DriveMotors drive;
    IAction<Boolean> actionExecutor;
    RobotController robot;
    InputResponseManager inputResponseManager;

    @Override
    public void init() {
        robot = new RobotController();
        drive = DriveMotors.fromMapDcMotor(hardwareMap.dcMotor, true, MotorSpec.GOBILDA_5203_2402_0003,
                "motor1", "motor1", "motor1", "motor1"
        );

        actionExecutor = new LaunchAutoSequenceAction<>(
                testDrive(drive)
        );

        actionExecutor.init(robot, false);


        drive.ul().setPower(gamepad1.dpad_up?1:0);
        drive.ur().setPower(gamepad1.dpad_right?1:0);
        drive.dr().setPower(gamepad1.dpad_down?1:0);
        drive.dl().setPower(gamepad1.dpad_left?1:0);

        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        if (x != 0 || y != 0) {
            drive.ul().setPower(0.5f * (x + y));
            drive.ur().setPower(0.5f * (x-y));
            drive.dl().setPower(0.5f * (x - y));
            drive.dr().setPower(0.5f * (x + y));

        }

        inputResponseManager = new InputResponseManager.Builder(new GamepadWrapper(gamepad1), robot)
                .AAction(actionExecutor)
                .build();

    }

    @Override
    public void loop() {
        inputResponseManager.loop();
    }
}
