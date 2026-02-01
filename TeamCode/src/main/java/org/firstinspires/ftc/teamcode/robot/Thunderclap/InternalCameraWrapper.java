package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.ObeliskPattern;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.util.LazyInit;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.robot.Thunderclap.SpindexerController.BallState;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

public class InternalCameraWrapper implements WithTelemetry.IWithTelemetry {

    final private VisionPortal visionPortal;
    final private AprilTagProcessor aprilTag;

    final private HashMap<Integer, ObeliskPattern<SpindexerController.BallState>> obeliskAprilTags;
    private ObeliskPattern<BallState> lastReadObeliskPattern;
    public Optional<ObeliskPattern<BallState>> getLastReadObeliskPattern() {
        if (lastReadObeliskPattern != null) return Optional.of(lastReadObeliskPattern);
        return Optional.empty();
    }

    boolean isReading;
    AprilTagDetection lastReading;

    final LazyInit<IAction<Object>> cameraDetect;
    public IAction<Object> cameraDetectAction() {return cameraDetect.get();}

    public InternalCameraWrapper(@NonNull HardwareMap hardwareMap) {

        obeliskAprilTags = new HashMap<>(3);
        obeliskAprilTags.put(21, new ObeliskPattern<>(
                BallState.GREEN,
                BallState.PURPLE,
                BallState.PURPLE
                ));
        obeliskAprilTags.put(22, new ObeliskPattern<>(
                BallState.PURPLE,
                BallState.GREEN,
                BallState.PURPLE
        ));
        obeliskAprilTags.put(23, new ObeliskPattern<>(
                BallState.PURPLE,
                BallState.PURPLE,
                BallState.GREEN
        ));

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                /*.setLensIntrinsics(
                        732.987, 732.987,
                        662.251, 363.643
                )*/
                .build();
        //aprilTag.setDecimation(2);-
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                //.setCameraResolution(new Size(1600, 1200))
                //.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        isReading = false;

        cameraDetect = new LazyInit<>(() -> IAction.From.loop((r, o) -> {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            if (currentDetections.isEmpty()) {
                isReading = false;
                return;
            }

            isReading = true;
            for (AprilTagDetection detection : currentDetections) {
                lastReading = detection;
                if (obeliskAprilTags.containsKey(detection.id)) {
                    lastReadObeliskPattern = obeliskAprilTags.get(detection.id);
                }
            }

        }));

        telemetryAction = new LazyInit<>(() -> WithTelemetry.fromLambda(() -> "Camera", telemetry -> {
            if (isReading) {
                telemetry.addData("Distance", lastReading.ftcPose.range);
                telemetry.addData("Rotation", lastReading.ftcPose.yaw);
            }
            if (lastReadObeliskPattern != null) {
                telemetry.addData("Last read obelisk pattern", lastReadObeliskPattern.toString());
            }
        }));
    }

    final LazyInit<IAction<Telemetry>> telemetryAction;

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction.get();
    }
}
