package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public interface IMotorBuildOpt <MotorType extends IMotor> {
    MotorType fromMap(HardwareMap.DeviceMapping<DcMotor> _map);
}
