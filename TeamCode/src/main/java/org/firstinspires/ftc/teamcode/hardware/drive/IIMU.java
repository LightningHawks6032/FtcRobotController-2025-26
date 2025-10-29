package org.firstinspires.ftc.teamcode.hardware.drive;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;


// Theres probably something better to call this lol
public interface IIMU extends WithTelemetry.IWithTelemetry {
    YawPitchRollAngles getAngles();
    AngularVelocity getVelocity();
    void resetYaw();
}
