package org.firstinspires.ftc.teamcode.opmodes.teleop.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.ServoWrapper;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.ThunderclapRobot;
import org.firstinspires.ftc.teamcode.util.Toggle;

import java.lang.annotation.Annotation;

@TeleOp(name = "Zak!!!! Kicker Tester")
public class ZaksKickerTester extends OpMode {
    ThunderclapRobot robot;

    ServoWrapper servo;
    Toggle toggle;
    @Override
    public void init() {
        servo = new ServoWrapper(hardwareMap.servo.get("kicker"));
        toggle = new Toggle();

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo.setPosition(0.3f);
        }
        if (gamepad1.b) {
            servo.setPosition(0.71f);
        }
        if (gamepad1.left_stick_y != 0) {
            servo.setVelocity(0.1f * gamepad1.left_stick_y);
        }
    }
}
