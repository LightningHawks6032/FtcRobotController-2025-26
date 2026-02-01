package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.auto.action.AutoActionSequence;
import org.firstinspires.ftc.teamcode.auto.action.ElapsedContainer;
import org.firstinspires.ftc.teamcode.auto.action.IActionAutoAction;
import org.firstinspires.ftc.teamcode.components.DirectDrive;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.components.action.EmptyAction;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.components.action.LaunchAutoSequenceAction;
import org.firstinspires.ftc.teamcode.components.action.PredicateAction;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.IMotor;
import org.firstinspires.ftc.teamcode.hardware.InternalIMUWrapper;
import org.firstinspires.ftc.teamcode.hardware.MotorSpec;
import org.firstinspires.ftc.teamcode.hardware.ServoWrapper;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.hardware.drive.IIMU;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.IOdometry;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.PinpointOdometry;
import org.firstinspires.ftc.teamcode.util.ButtonCounter;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ThunderclapRobot implements IRobot {

    final IOdometry odometry;
    final IIMU imu;
    public final DriveMotors driveMotors;

    public final DirectDrive directDrive;
    public final StateMachineDrive stateMachineDrive;
    public final OuttakeWheelController outtakeController;

    public final IntakeWheelController intakeController;

    //public final TransferWheelController transferController;

    public final HoodController hoodController;

    public final InternalCameraWrapper camera;

    public final IAction<Boolean> resetHeadingAction;

    public TransferWheelController transferController;

    public final ColorSensorWrapper colorSensor;
    public final SpindexerController.TargetPositionCommander spindexerCommander;
    public SpindexerController.SpindexerPositionController spindexerController;
    public final SpindexerController.BallStateDeterminer ballStateDeterminer;
    public final KickerController kickerController;
    public final IntakeSwingController intakeSwingController;
    public final IntakeAutomationController intakeAutomationController;
    public ButtonCounter leftSpindexer, rightSpindexer;

    @Override
    public DriveMotors getDrive() {
        return driveMotors;
    }

    @Override
    public IOdometry getOdometry() {
        return odometry;
    }

    @Override
    public IIMU getIMU() {
        return imu;
    }

    float heading = 0f;

    public ThunderclapRobot(@NonNull HardwareMap hardwareMap) {
        imu = new InternalIMUWrapper(hardwareMap.get(IMU.class, "imu"));

        driveMotors = DriveMotors.fromMapDcMotor(hardwareMap.dcMotor,
                false,
                MotorSpec.GOBILDA_5203_2402_0019,
                "fl", "fr", "br", "bl"
        );

        driveMotors.ur().setDirection(IMotor.Direction.REVERSE);
        driveMotors.dl().setDirection(IMotor.Direction.FORWARD);
        driveMotors.dr().setDirection(IMotor.Direction.REVERSE);
        driveMotors.ul().setDirection(IMotor.Direction.FORWARD);

        driveMotors.setEncoderPositionSignMap(new DriveMotors.EncoderPositionSignMap(
                false,
                false,
                false,
                false
        ));

        odometry = new PinpointOdometry(hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"));

        directDrive = new DirectDrive(driveMotors, DirectDrive.fieldCentricFromIMUGamepad(imu));
        directDrive.setDrivePowerFactor(0.8f);


        camera = new InternalCameraWrapper(hardwareMap);
        stateMachineDrive = new StateMachineDrive(directDrive,
                new PIDF.BuildOpt(new PIDF.Weights(
                1f, 0.1f,0.05f,0f,0.01f,1)), () -> {
            float fallback = 0f;
            AprilTagDetection last = camera.lastReading;
            if (last == null) {return fallback;}
            return (float)last.ftcPose.yaw / 180 * (float)Math.PI/*robotPose.getOrientation().getYaw(AngleUnit.RADIANS)*/;
        }, () -> odometry.getPos().r - heading, () -> camera.isReading,
                odometry::getPos,
                () -> {
                AprilTagDetection last = camera.lastReading;
                return last == null || last.ftcPose == null ? 100f : (float)last.ftcPose.range;
                })
        ;

        outtakeController = new OuttakeWheelController(
                Util.also(new DcMotorWrapper(hardwareMap.dcMotor.get("supplementary flywheel"), true, MotorSpec.GOBILDA_5002_0002_0001), m ->
                    m.setDirection(IMotor.Direction.FORWARD)),
                    new PIDF.BuildOpt(new PIDF.Weights(
                                0.9f,
                                0.7f,0.25f,
                                1f,
                                0.1f,1
                    )),
                () -> camera.lastReading
                );

        outtakeController.setSupplementaryMotor(
                Util.also(new DcMotorWrapper(hardwareMap.dcMotor.get("outtake flywheel"), true, MotorSpec.GOBILDA_5002_0002_0001),
                        m->m.setDirection(IMotor.Direction.REVERSE))
        );

        hoodController = new HoodController(
                new ServoWrapper(hardwareMap.servo.get("hood")),
                0.4f, 0.8f,
                () -> {
                    AprilTagDetection last = camera.lastReading;
                    if (last == null) {
                        return 100f;
                    }
                    return (float)last.ftcPose.range;
                }
        );

        intakeController = new IntakeWheelController(
                Util.also(new DcMotorWrapper(hardwareMap.dcMotor.get("intake flywheel"), false, MotorSpec.GOBILDA_5203_2402_0019),
                        it -> it.setDirection(IMotor.Direction.REVERSE))
        );

        /*transferController = new TransferWheelController(
                new ServoWrapper(hardwareMap.servo.get("transfer flywheel")),
                intakeController::trySyncTransferLift
        );*/

        colorSensor = new ColorSensorWrapper(hardwareMap.get(NormalizedColorSensor.class, "color sensor"));
        colorSensor.setGain(10);

        leftSpindexer = new ButtonCounter();
        rightSpindexer = new ButtonCounter();

        spindexerCommander = new SpindexerController.TargetPositionCommander();
        spindexerController = new SpindexerController.SpindexerPositionController(
                new PIDF.Weights(/*50,1.2f,1*/0.03f, 0.0f, 0.000f,0,0f,1),
                Util.also(new DcMotorWrapper(hardwareMap.dcMotor.get("spindexer"), true,
                        MotorSpec.GOBILDA_5203_2402_0019), m -> m.setDirection(IMotor.Direction.FORWARD)),
                () -> {
                    int idx = leftSpindexer.count() - rightSpindexer.count();
                    if (idx < 0) {
                        idx += -3 * idx;
                    }
                    idx %= 3;
                    return spindexerCommander.getPosFromIndex(idx);
                }
        );

        ballStateDeterminer = new SpindexerController.BallStateDeterminer(colorSensor::getColor);



        kickerController = new KickerController(new ServoWrapper(hardwareMap.servo.get("kicker")));

        intakeSwingController = new IntakeSwingController(new ServoWrapper(hardwareMap.servo.get("intake servo")));

        intakeAutomationController = new IntakeAutomationController(this);

        resetHeadingAction = new PredicateAction<>(
                IAction.From.loop((r, b) ->
                {
                    heading = odometry.getPos().r;
                    imu.resetYaw();
                }),
                new EmptyAction<>(),
                (r, b) -> b
        );
    }
}
