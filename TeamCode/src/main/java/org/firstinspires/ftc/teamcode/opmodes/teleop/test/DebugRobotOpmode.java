package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.TimerWrapper;

import java.util.Iterator;
@TeleOp(name="Debug Robot Motors")
public class DebugRobotOpmode extends OpMode {

    float waitTime = 2f;
    boolean isOn = false;
    TimerWrapper timer;

    Iterator<DcMotor> dcMotorIterator;
    DcMotor curr;
    @Override
    public void init() {
        dcMotorIterator = hardwareMap.dcMotor.iterator();
        timer = new TimerWrapper();
        curr = dcMotorIterator.next();
    }

    @Override
    public void loop() {
        if (timer.get() > waitTime) {
            isOn = !isOn;
            if (!isOn) {
                if (dcMotorIterator.hasNext()) curr = dcMotorIterator.next();
            }
            curr.setPower(isOn ? 1 : 0);

        }
    }
}
