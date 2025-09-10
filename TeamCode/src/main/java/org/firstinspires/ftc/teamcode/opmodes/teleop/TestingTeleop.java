package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;

@TeleOp(name = "TestingTeleop", group = "Testing")
public class TestingTeleop extends OpMode {

    DcMotorWrapper motor;
    PIDF posC, velC;

    float targetPosition = 0;
    float targetVelocity = 0;
    float targetPower = 0;

    float I, D;
    @Override
    public void init() {
        motor = new DcMotorWrapper(hardwareMap.dcMotor.get("motor1"), true, null);
        posC = new PIDF(new PIDF.Weights(1f, 0f, 0.01f, 0, 0.75f, 0.2f));
        posC.setRateLimit(0.05f);

        velC = new PIDF(new PIDF.Weights(0.1f, 0, 0, 1, 0.75f, 0.15f))
                .setRateLimit(0.05f);
    }

    void rs() {
        posC.reset(motor.getPosition());
        velC.reset(0);
        targetPosition = motor.getPosition();
    }

    @Override
    public void loop() {



        if (gamepad1.dpad_right) {
            I += 0.01f;
            velC.setWeights(new PIDF.Weights(0.1f, I, D, 1, 0.75f, 0.15f));
            rs();
        }
        if (gamepad1.dpad_left) {
            I -= 0.01f;
            velC.setWeights(new PIDF.Weights(0.1f, I, D, 1, 0.75f, 0.15f));
            rs();
        }
        if (gamepad1.dpad_up) {
            D += 0.01f;
            velC.setWeights(new PIDF.Weights(0.1f, I, D, 1, 0.75f, 0.15f));
            rs();
        }
        if (gamepad1.dpad_down) {
            D -= 0.01f;
            velC.setWeights(new PIDF.Weights(0.1f, I, D, 1, 0.75f, 0.15f));
            rs();
        }
        if (gamepad1.x) {
            rs();
        }

        targetPosition += gamepad1.left_stick_y * 30;

        posC.setTarget(targetPosition);
        targetVelocity = posC.loop((float)motor.getPosition()) / 6000;

        velC.setTarget(targetVelocity);
        targetPower = velC.loop(motor.encoder.getVelocity() / 6000);



        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("target", targetPosition);
        telemetry.addData("current", motor.getPosition());
        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("currentVelocity", motor.encoder.getVelocity());
        telemetry.addData("power", targetPower);

        motor.setPower(targetPower * 0.6f);
    }
}
