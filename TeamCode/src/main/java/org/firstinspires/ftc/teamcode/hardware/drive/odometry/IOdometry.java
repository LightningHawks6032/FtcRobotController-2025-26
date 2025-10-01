package org.firstinspires.ftc.teamcode.hardware.drive.odometry;

import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

public interface IOdometry extends WithTelemetry.IWithTelemetry {
    void loop(float dt);
    void setPos(Vec2Rot pos);
    Vec2Rot getPos();
    Vec2Rot getVel();
    Vec2Rot getAcc();
}
