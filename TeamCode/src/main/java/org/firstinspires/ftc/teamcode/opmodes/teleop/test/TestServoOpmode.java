package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServoOpmode extends OpMode {
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        servo.setPosition(0);
    }

    @Override
    public void start() {
        servo.setPosition(1);
    }

    @Override
    public void loop() {

    }
}
