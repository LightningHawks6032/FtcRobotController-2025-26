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
        posC = new PIDF(new PIDF.Weights(1f, 0f, 0.4f, 0, 0.75f, 0.2f));
        posC.setRateLimit(0.05f);

        velC = new PIDF(new PIDF.Weights(0.1f, 0, 0.5f, 1, 0.75f, 0.15f))
                .setRateLimit(0.05f);
    }

    void rs() {
        posC.reset(motor.getPosition());
        velC.reset(0);
        targetPosition = motor.getPosition();
    }


    float stall_torque =  1.47f * 9.80665e-3f;
    float no_load_speed = 6000;
    float encoder_resolution = 145.6f;
    float omega_no_load = no_load_speed * 2 * 3.14f / 60;
    float ticks_per_rad = encoder_resolution / (2 * 3.14f);
    float j = stall_torque / 50;

    float position_deadband_ticks = 2;
    float integrator_clamp = 1e4f;
    float friction = 0;
    float max_cmd = 1;

    @Override
    public void loop() {
        targetPosition += gamepad1.left_stick_y * 30;


        {
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
        }

        float error = targetPosition - motor.getPosition();

        // deadband
        if (Math.abs(error) <= position_deadband_ticks) {
            targetVelocity = 0;
        }
        else {
            posC.setTarget(targetPosition);
            targetVelocity = posC.loop((float)motor.getPosition());
        }

        velC.setTarget(targetVelocity);
        targetPower = velC.loop(motor.encoder.getVelocity());

        // TODO: clamp integrators
        float torque;
        if (Math.abs(targetPower) > 1e-6) {
            float omega_limit = Math.abs(targetPower) * omega_no_load;
            float torque_limit = Math.abs(targetPower) * stall_torque;
            float frac = Math.max(0, 1-Math.abs(motor.encoder.getVelocity() / ticks_per_rad) / omega_limit);
            torque = Math.copySign(1, targetPower) * torque_limit * frac;
        }
        else {
            torque = 0;
        }

        // friction
        float omega_rad = motor.encoder.getVelocity() / ticks_per_rad;
        float net_torque = torque - friction * omega_rad;

        float alpha = net_torque / j;

        float omega = (motor.encoder.getVelocity() / ticks_per_rad) + alpha * 0.05f;
        float theta = (motor.getPosition() / ticks_per_rad) + omega * 0.05f;

        float new_velocity_ticks = omega * ticks_per_rad;
        float new_position_ticks = theta * ticks_per_rad;

        // encoder quantization
        float measured_position;
        float quantized = (float)Math.floor(Math.round(new_position_ticks));
        if (Math.abs(quantized - Math.floor(Math.round(motor.getPosition()))) >= 1) {
            measured_position = quantized;
        }
        else {
            measured_position = motor.getPosition();
        }

        //motor.setPower()

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
