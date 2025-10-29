package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.IRobot;
import org.firstinspires.ftc.teamcode.hardware.GamepadWrapper;
import org.firstinspires.ftc.teamcode.hardware.InputResponseManager;
import org.jetbrains.annotations.Contract;

import java.util.function.BiFunction;
import java.util.function.Function;

public class TeleOpmode <TyRobot extends IRobot> {
    InputResponseManager g1, g2;


    public void loop() {
        g1.loop();
        g2.loop();
    }

    public TeleOpmode(GamepadWrapper gamepad1,
                      GamepadWrapper gamepad2,
                      TyRobot robot,
                      Telemetry telemetry,
                      @NonNull BiFunction<TyRobot, InputResponseManager.Builder, InputResponseManager> gamepad1Builder,
                      @NonNull BiFunction<TyRobot, InputResponseManager.Builder, InputResponseManager> gamepad2Builder) {
        g1 = gamepad1Builder.apply(robot, new InputResponseManager.Builder(gamepad1, robot, telemetry));
        g2 = gamepad2Builder.apply(robot, new InputResponseManager.Builder(gamepad2, robot, telemetry));
    }

    public TeleOpmode(@NonNull OpMode opmode,
                      TyRobot robot,
                      @NonNull BiFunction<TyRobot, InputResponseManager.Builder, InputResponseManager> gamepad1Builder,
                      @NonNull BiFunction<TyRobot, InputResponseManager.Builder, InputResponseManager> gamepad2Builder) {

        this(
                new GamepadWrapper(opmode.gamepad1),
                new GamepadWrapper(opmode.gamepad2),
                robot,
                opmode.telemetry,
                gamepad1Builder,
                gamepad2Builder
        );
    }

    @NonNull
    @Contract(pure = true)
    public static <_TyRobot> BiFunction<_TyRobot, InputResponseManager.Builder, InputResponseManager> EmptyGamepad()
    {
        return (r, b) -> b.build();
    }
}
