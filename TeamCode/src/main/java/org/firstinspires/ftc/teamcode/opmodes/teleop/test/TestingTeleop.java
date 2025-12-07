package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name = "TestingTeleop", group = "Testing")
public class TestingTeleop extends OpMode {

    DcMotorWrapper motor;
    PIDF.Controller posC, velC;

    float targetPosition = 0;
    final float targetVelocity = 0;
    final float targetPower = 0;

    float I, D;
    @Override
    public void init() {
        motor = new DcMotorWrapper(hardwareMap.dcMotor.get("motor1"), true, null);
        posC = new PIDF.Controller(new PIDF.Weights(1f, 0f, 0.4f, 0, 0.29f, 0.2f));

        velC = new PIDF.Controller(new PIDF.Weights(0.1f, 0, 0.5f, 1, 0.75f, 0.15f));
    }

    void rs() {
        targetPosition = motor.getPosition();
    }


    final float stall_torque =  1.47f * 9.80665e-3f;
    final float no_load_speed = 6000;
    final float encoder_resolution = 145.6f;
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
