package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.util.TimerWrapper;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.Vec2;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

import java.io.IOException;
import java.util.function.Supplier;

public class PIDDrive implements WithTelemetry.IWithTelemetry {
    DataLogger logger;
    TimerWrapper timer;


    PIDF.Controller pidR;

    Vec2 dirXY;
    float dirR;
    float rSpeed;
    float targetR;


    public IAction<Vec2> setXY;
    public IAction<Float> setR;

    public IAction<Float> driveLoop;

    Supplier<Float> getYaw;

    public PIDDrive(Supplier<Float> _getYaw, DirectDrive _drive, PIDF.Weights rWeights) {
        timer = new TimerWrapper();

        try {
            logger = new DataLogger("PID DRIVE LOG");
            logger.addHeaderLine("Time", "Target R", "Current R", "PID Current", "PID Target", "PID Output");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }


        getYaw = _getYaw;
        pidR = new PIDF.Controller(rWeights);

        setXY = IAction.From.loop((r, v) -> dirXY = v);
        setR = IAction.From.loop((r, f) -> dirR = f);

        rSpeed = 3f;

        driveLoop = IAction.From.loop((r, f) -> {
            targetR = Util.normAngle(targetR + rSpeed * dirR * f);
            float rCmd = 0.6f * Util.clamp(pidR.loop(Util.normAngle(targetR - getYaw.get()), 0f, f), -1, 1);

            try {
                logger.addDataLine(timer.get(), targetR, getYaw.get(), Util.normAngle(targetR - getYaw.get()), 0f, rCmd);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

            _drive.splitAction.leftSetter().loop(r, dirXY);
            _drive.splitAction.rightSetter().loop(r, new Vec2(rCmd, 0));

            _drive.splitAction.loop(r, 0);

        });
    }

    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return WithTelemetry.fromLambda(() -> "PID Drive", telemetry -> {
            telemetry.addData("target R", targetR);
        });
    }
}
