package org.firstinspires.ftc.teamcode.hardware.drive.odometry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.util.LazyInit;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

public class PinpointOdometry implements IOdometry{
    GoBildaPinpointDriver pinpoint;

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         *///
        pinpoint.setOffsets(4 + 3f/16f + 1.5f, 0, DistanceUnit.CM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

    public PinpointOdometry(GoBildaPinpointDriver _pinpoint) {
        pinpoint = _pinpoint;

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        telemetryAction = new LazyInit<>(() -> WithTelemetry.fromLambda(() -> "Pinpoint Odometry", telemetry -> {
            telemetry.addData("Position", getPos().toString());
            telemetry.addData("Velocity", getVel().toString());
        }));
    }

    @Override
    public void loop(float dt) {
        pinpoint.update();
    }

    @Override
    public void setPos(Vec2Rot pos) {
        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.CM,
                        pos.x, pos.y,
                        AngleUnit.RADIANS,
                        pos.r
                )
        );
    }

    @Override
    public Vec2Rot getPos() {
        Pose2D pos = pinpoint.getPosition();
        return new Vec2Rot(
                (float) pos.getX(DistanceUnit.CM),
                (float) pos.getY(DistanceUnit.CM),
                (float) pos.getHeading(AngleUnit.RADIANS)
        );
    }

    @Override
    public Vec2Rot getVel() {
        return new Vec2Rot(
                (float)pinpoint.getVelX(DistanceUnit.CM),
                (float)pinpoint.getVelY(DistanceUnit.CM),
                (float)pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        );
    }

    @Override
    public Vec2Rot getAcc() {
        return Vec2Rot.zero();
    }

    @Override
    public void resetHeading() {
        pinpoint.setHeading(0, AngleUnit.RADIANS);
    }

    LazyInit<IAction<Telemetry>> telemetryAction;

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction.get();
    }
}
