package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.util.ColorRGBA;
import org.firstinspires.ftc.teamcode.util.LazyInit;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;


public class ColorSensorWrapper implements WithTelemetry.IWithTelemetry {

    NormalizedColorSensor sensor;
    LazyInit<IAction<Telemetry>> telemetryAction;

    public ColorSensorWrapper(NormalizedColorSensor _sensor) {
        sensor = _sensor;
        telemetryAction = new LazyInit<>(() ->
            WithTelemetry.fromLambda(() -> "Color Sensor Wrapper", telemetry -> {
                telemetry.addData("Gain", getGain());
                telemetry.addData("Reading", getColor().toString());
                telemetry.addData("Distance", getDistance());
            })
        );
    }

    public float getGain() {return sensor.getGain();}
    public void setGain(float _gain) {sensor.setGain(_gain);}
    public ColorRGBA getColor() {
        NormalizedRGBA color = sensor.getNormalizedColors();
        return new ColorRGBA(color.red, color.green, color.blue, color.alpha);
    }

    public float getDistance() {
        if (sensor instanceof DistanceSensor) {
            return (float)((DistanceSensor)sensor).getDistance(DistanceUnit.CM);
        }
        return 0f;
    }


    @Override
    public IAction<Telemetry> getTelemetryAction() {
        return telemetryAction.get();
    }
}
