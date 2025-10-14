package org.firstinspires.ftc.teamcode.hardware.drive.odometry.deadwheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DeadwheelWrapper {
    DcMotorEx deadwheel;

    public int getPosition() {
        return deadwheel.getCurrentPosition();
    }

    public DeadwheelWrapper(DcMotorEx _deadwheel) {
        deadwheel = _deadwheel;
    }
}
