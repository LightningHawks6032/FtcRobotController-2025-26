package org.firstinspires.ftc.teamcode.hardware.drive.odometry.deadwheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DeadwheelWrapper {
    DcMotorEx deadwheel;
    boolean reversed = false;
    public void setReverse(boolean rev) {
        reversed = rev;
    }

    public int getPosition() {
        return deadwheel.getCurrentPosition() * (reversed?-1:1);
    }

    public DeadwheelWrapper(DcMotorEx _deadwheel) {
        deadwheel = _deadwheel;
    }
}
