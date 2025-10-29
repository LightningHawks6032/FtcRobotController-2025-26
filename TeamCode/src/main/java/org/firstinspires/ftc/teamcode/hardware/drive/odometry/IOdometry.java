package org.firstinspires.ftc.teamcode.hardware.drive.odometry;

import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

public interface IOdometry extends WithTelemetry.IWithTelemetry {
    void loop(float dt);
    void setPos(Vec2Rot pos);
    Vec2Rot getPos();
    Vec2Rot getVel();
    Vec2Rot getAcc();
    void resetHeading();

    default IAction<Float> getLoopAction() {
        return new IAction<Float>() {

            @Override
            public void init(IRobot robot, Float data) {

            }

            @Override
            public void start(IRobot robot, Float data) {

            }

            @Override
            public void loop(IRobot robot, Float data) {
                IOdometry.this.loop(data);
            }
        };
    }
    default IAction<Vec2Rot> getAssertPositionAction() {
        return IAction.From.loop((r, v) -> setPos(v));
    }
}
