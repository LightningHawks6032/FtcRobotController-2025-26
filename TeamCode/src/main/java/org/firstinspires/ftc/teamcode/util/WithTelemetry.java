package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.RobotController;
import org.firstinspires.ftc.teamcode.components.action.IAction;

public class WithTelemetry {
    public interface ITelemetry extends WithTelemetry.IWithTelemetry{
        String getName();
        void loop(Telemetry _telemetry);
    }
    public static class Action <Ty extends ITelemetry> implements IAction<Telemetry> {

        Ty telemetry;

        void telemFmt(@NonNull Telemetry telem) {
            telem.addLine("------- " + telemetry.getName() + " -------");
            telemetry.loop(telem);
        }

        public Action(Ty _telemetry) {
            telemetry = _telemetry;
        }

        @Override
        public void init(RobotController robot, Telemetry data) {
            telemFmt(data);
        }

        @Override
        public void start(RobotController robot, Telemetry data) {
            telemFmt(data);
        }

        @Override
        public void loop(RobotController robot, Telemetry data) {
            telemFmt(data);
        }
    }

    public interface IWithTelemetry {
        IAction<Telemetry> getTelemetryAction();
    }
}
