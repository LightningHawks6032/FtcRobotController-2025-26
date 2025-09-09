package org.firstinspires.ftc.teamcode.hardware;

public class MotorSpec {
    float noLoadSpeed;
    float stallTorque;
    float encoderResolution;
    public MotorSpec(float _noLoadSpeed_RPM, float _stallTorque_KgCm, float _encoderResolution_PPR) {
        noLoadSpeed = _noLoadSpeed_RPM;
        stallTorque = _stallTorque_KgCm;
        encoderResolution = _encoderResolution_PPR;
    }
}
