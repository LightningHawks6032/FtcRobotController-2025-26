package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "Webcam Test", group = "Testing")
public class WebcamTest extends OpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    public void initAprilTag(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();
        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();


    }

    @Override
    public void init() {
        initAprilTag(hardwareMap);
    }

    @Override
    public void loop() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.isEmpty()) {
            telemetry.addLine("No apriltag found");
        }

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (true || detection.metadata != null) {
                telemetry.addData("Pose", detection.robotPose.toString());
                telemetry.addData("Distance", detection.ftcPose.range);
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
    }
}
