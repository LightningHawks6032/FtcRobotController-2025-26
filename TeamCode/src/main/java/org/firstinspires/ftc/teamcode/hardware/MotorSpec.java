package org.firstinspires.ftc.teamcode.hardware;

public class MotorSpec {
    float noLoadSpeed;
    float stallTorque;
    float encoderResolution;
    float gearRatio;
    public MotorSpec(float _noLoadSpeed_RPM, float _stallTorque_KgCm, float _encoderResolution_PPR, float _gearRatio) {
        noLoadSpeed = _noLoadSpeed_RPM;
        stallTorque = _stallTorque_KgCm;
        encoderResolution = _encoderResolution_PPR;
        gearRatio = _gearRatio;
    }

    // where did this come from :skull:
    public static MotorSpec GOBILDA_5000_0002_0001 = new MotorSpec(6000, 1.47f, 145.6f, 1);
    public static MotorSpec GOBILDA_5203_2402_0003 = new MotorSpec(1620, 5.4f, 103.8f, 3.7f);
    public static MotorSpec GOBILDA_5203_2402_0019 = new MotorSpec(312, 24.3f, 537.7f, 19.2f);



}
