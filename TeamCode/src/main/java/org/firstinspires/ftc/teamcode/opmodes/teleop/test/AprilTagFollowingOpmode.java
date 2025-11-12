package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.DirectDrive;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.PredicateAction;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.drive.IIMU;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpmode;
import org.firstinspires.ftc.teamcode.robot.ClankerHawk2A;
import org.firstinspires.ftc.teamcode.util.TimerWrapper;
import org.firstinspires.ftc.teamcode.util.Vec2Rot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "April Tag Following Opmode", group = "Testing")
public class AprilTagFollowingOpmode extends OpMode {

    ClankerHawk2A robot;
    TeleOpmode<ClankerHawk2A> teleop;
    CameraTracking tracking;
    DirectDrive driveAction;
    PIDF.Controller pid;
    class CameraTracking {
        private VisionPortal visionPortal;
        private AprilTagProcessor aprilTag;
        private final IIMU imu;
        public float lastAngle;
        public float lastAngleIMU;
        public boolean readTag;
        public CameraTracking(HardwareMap map, IIMU _imu) {
            lastAngle = 0;
            lastAngleIMU = 0;
            readTag = false;
            imu = _imu;
            initAprilTag(map);
        }

        void findAngle(@NonNull AprilTagDetection tag) {
            lastAngle = (float)tag.ftcPose.yaw;
            lastAngleIMU = (float)imu.getAngles().getYaw();
        }

        public void initAprilTag(@NonNull HardwareMap hardwareMap) {
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                    .build();
            aprilTag.setDecimation(2);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();

        }
        public void loop() {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            if (currentDetections.isEmpty()) {
                telemetry.addLine("No apriltag found");
            }

            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                readTag = true;
                telemetry.addData("yaw", detection.ftcPose.yaw);
                telemetry.addData("roll", detection.ftcPose.roll);
                telemetry.addData("pitch", detection.ftcPose.pitch);
                findAngle(detection);
            }
        }
    }
    TimerWrapper timer;
    @Override
    public void init() {
        pid = new PIDF.Controller(
                new PIDF.Weights(0.0035f, 0f, 0.1f, 1, 0.3f, 0.1f)
        );

        robot = new ClankerHawk2A(hardwareMap);
        tracking = new CameraTracking(hardwareMap, robot.getIMU());
        timer = new TimerWrapper();
        driveAction = new DirectDrive(robot.driveMotors,
                (v1, v2) ->
                {
                    return new Vec2Rot(0, 0, pid.loop(
                            (float)robot.getIMU().getAngles().getYaw(),
                            tracking.lastAngleIMU,
                            timer.get()
                            ));
                }

                );

        teleop = new TeleOpmode<>(
                this, robot,
                (robot, b) -> b
                        .leftStickAction(
                                driveAction.splitAction().leftSetter(),
                                robot.directDrive.splitAction().leftSetter()
                        )
                        .rightStickAction(
                                driveAction.splitAction().rightSetter(),
                                robot.directDrive.splitAction().rightSetter()
                        )
                        .XAction(
                                robot.resetHeadingAction
                        )
                        .telemetry(
                                robot.getOdometry(),
                                robot.getDrive(),
                                robot.getIMU()
                        )
                        .loops(
                                new PredicateAction<>(
                                        driveAction.splitAction(),
                                        robot.directDrive.splitAction(),
                                        (r, o) -> gamepad1.right_stick_button
                                )
                        )
                        .timeLoops(
                                robot.getOdometry().getLoopAction()
                        )
                        .build(),
                TeleOpmode.EmptyGamepad()
        );
    }

    @Override
    public void loop() {
        tracking.loop();
        teleop.loop();
        timer.reset();
    }
}
