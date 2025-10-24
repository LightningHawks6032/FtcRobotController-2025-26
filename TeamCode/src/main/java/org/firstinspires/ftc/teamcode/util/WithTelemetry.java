package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.RobotController;
import org.firstinspires.ftc.teamcode.components.action.ActionGroup;
import org.firstinspires.ftc.teamcode.components.action.IAction;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class WithTelemetry {
    public interface ITelemetry{
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

    public static IAction<Telemetry> fromLambda(Supplier<String> _getName, Consumer<Telemetry> _loop) {
        return new WithTelemetry.Action<WithTelemetry.ITelemetry>(new WithTelemetry.ITelemetry() {
            @Override
            public String getName() {
                return _getName.get();
            }

            @Override
            public void loop(@NonNull Telemetry _telemetry) {
                _loop.accept(_telemetry);
            }
        });
    }

    public static IAction<Telemetry> header(Supplier<String> _getName) {
        return new WithTelemetry.Action<WithTelemetry.ITelemetry>(new WithTelemetry.ITelemetry() {
           @Override
           public String getName() {return _getName.get();}

           @Override
           public  void loop(@NonNull Telemetry _telemetry) {}
        });
    }

    public static IAction<Telemetry> withHeader(Supplier<String> _getName, IAction<Telemetry> _action) {
        return new ActionGroup<Telemetry>(
                header(_getName),
                _action
        );
    }
}
