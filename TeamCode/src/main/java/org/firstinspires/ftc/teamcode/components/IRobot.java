package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.hardware.drive.IIMU;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.IOdometry;

public interface IRobot {
    DriveMotors getDrive();
    IOdometry getOdometry();
    IIMU getIMU();

}
