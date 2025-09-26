package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.DebugMotor;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;

@TeleOp(name = "Outtake Motor Test", group = "Testing")
public class OuttakeMotorTestOpmode extends OpMode {
    DcMotorWrapper motor1;
    DebugMotor motor2;

    float motor_power;
    boolean lock;


    @Override
    public void init() {
        DcMotor temp = hardwareMap.dcMotor.get("motor1");
        temp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor1 = new DcMotorWrapper(temp, true, MotorSpec.GOBILDA_5203_2402_0003);
        motor2 = new DebugMotor("yo", telemetry, null);//(DcMotorEx) hardwareMap.dcMotor.get("motor2");
        motor_power = 0;
        lock = false;
    }


    @Override
    public void loop() {
        // Sets the motor power to the left stick y position
        if (!lock) motor_power = gamepad1.left_stick_y;
        if (gamepad1.a) {
            lock = true;
        }

        if (gamepad1.b) {
            lock = false;
        }

        // Set power takes a value between -1.0 and 1.0
        motor1.setPower(motor_power);
        motor2.setPower(-motor_power);

        telemetry.addData("Motor 1 speed (rpm)", motor1.ticksPerSecondToRPM(motor1.encoder.getVelocity()));
        telemetry.addData("Motor 1 vel computer (ticks/s)", motor1.getVelocity());
        telemetry.addData("pow", motor1.getPower());
        telemetry.addData("Motor 1 vel measured (ticks/s)", motor1.encoder.getVelocity());
    }
}
