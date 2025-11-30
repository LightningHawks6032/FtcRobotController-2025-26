package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.hardware.drive.IIMU;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

public class InternalIMUWrapper implements IIMU {
    IMU imu;

    public IMU imu() {return imu;}

    public InternalIMUWrapper(IMU _imu, RevHubOrientationOnRobot orientation) {
        imu = _imu;

        imu.initialize(new IMU.Parameters(orientation));
    }

    public InternalIMUWrapper(IMU _imu) {
        imu = _imu;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    IAction<Telemetry> telemetryAction = WithTelemetry.fromLambda(() -> "IMU", (telem) -> {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        telem.addData("Yaw", angles.getYaw(AngleUnit.DEGREES));
        telem.addData("Pitch", angles.getPitch(AngleUnit.DEGREES));
        telem.addData("Roll", angles.getRoll(AngleUnit.DEGREES));
    });

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction;
    }

    @Override
    public YawPitchRollAngles getAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    @Override
    public AngularVelocity getVelocity() {return imu.getRobotAngularVelocity(AngleUnit.RADIANS);}

    @Override
    public void resetYaw() {
        imu.resetYaw();
    }
}
