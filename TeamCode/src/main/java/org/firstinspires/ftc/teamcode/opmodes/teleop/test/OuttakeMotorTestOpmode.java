package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Outtake Motor Test", group = "Testing")
public class OuttakeMotorTestOpmode extends OpMode {
    DcMotorEx motor1, motor2;

    float motor_power;

    @Override
    public void init() {
        motor1 = (DcMotorEx) hardwareMap.dcMotor.get("motor1");
        motor2 = (DcMotorEx) hardwareMap.dcMotor.get("motor2");
        motor_power = 0;
    }

    @Override
    public void loop() {
        // Sets the motor power to the left stick y position
        motor_power = gamepad1.left_stick_y;
        // Set power takes a value between -1.0 and 1.0
        motor1.setPower(motor_power);
        motor2.setPower(-motor_power);
        // Add rotation data to telemetry
        telemetry.addData("Motor 1 velocity (rad/s)", motor1.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Motor 2 velocity (rad/s)", motor2.getVelocity(AngleUnit.RADIANS));

    }
}
