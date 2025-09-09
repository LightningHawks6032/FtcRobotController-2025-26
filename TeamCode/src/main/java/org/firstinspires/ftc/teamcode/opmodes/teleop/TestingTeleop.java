package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.control.IControlLoop;
import org.firstinspires.ftc.teamcode.control.PID;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;

@TeleOp(name = "TestingTeleop", group = "Testing")
public class TestingTeleop extends OpMode {

    DcMotorWrapper motor;
    PID posC, velC;

    float targetPosition = 0;
    float targetVelocity = 0;
    float targetPower = 0;
    @Override
    public void init() {
        motor = new DcMotorWrapper(hardwareMap.dcMotor.get("motor1"), true, null);
        posC = new PID(0.1f, 0.15f, 0.1f);
        posC.setRateLimit(0.05f);

        velC = new PID(0.1f, 0.15f, 0.1f)
                .setRateLimit(0.05f);
    }

    @Override
    public void loop() {
        targetPosition += gamepad1.left_stick_y * 30;

        posC.setTarget(targetPosition);
        targetVelocity = posC.loop(targetPosition - (float)motor.getPosition());

        velC.setTarget(targetVelocity);
        targetPower = velC.loop(targetVelocity - motor.getVelocity());
        telemetry.addData("target", targetPosition);
        telemetry.addData("current", motor.getPosition());
        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("currentVelocity", motor.getVelocity());
        telemetry.addData("power", targetPower);

        motor.setPower(targetPower);
    }
}
