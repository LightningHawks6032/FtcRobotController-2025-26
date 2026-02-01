package org.firstinspires.ftc.teamcode.robot.Thunderclap;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.action.IAction;
import org.firstinspires.ftc.teamcode.control.AnglePositionPID;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.hardware.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.hardware.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.util.ColorRGBA;
import org.firstinspires.ftc.teamcode.util.LazyInit;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.WithTelemetry;

import java.lang.annotation.Target;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;

public class SpindexerController {
    public enum BallState {
        GREEN,
        PURPLE,
        NONE
    }



    public static class BallStateDeterminer implements WithTelemetry.IWithTelemetry {

        final Supplier<ColorRGBA> getColorReading;
        final HashMap<ColorRGBA, BallState> ballStateMapping;
        final LazyInit<IAction<Telemetry>> telemetryAction;


        public BallStateDeterminer(Supplier<ColorRGBA> _getColorReading) {
            getColorReading = _getColorReading;

            ballStateMapping = new HashMap<>(3);
            ballStateMapping.put(
                    new ColorRGBA(0.009613184f,0.014648661f,0.011444267f, 1), BallState.NONE);
            ballStateMapping.put(
                    new ColorRGBA(0.14145114f,0.15793088f,0.2668803f,1), BallState.PURPLE);
            ballStateMapping.put(
                    new ColorRGBA(0.057831693f,0.21484703f,0.1622034f,1), BallState.GREEN);

            telemetryAction = new LazyInit<>(() -> WithTelemetry.fromLambda(() -> "Ball State Determiner", telemetry -> {
                ColorRGBA reading = getColorReading.get();
                for (ColorRGBA key : ballStateMapping.keySet()) {
                    telemetry.addData(ballStateMapping.get(key).toString() + " distance", key.distanceSquared(reading));
                }
                telemetry.addData("Predicted State", getPredictedState().toString());
            }));
        }

        public BallState getPredictedState() {
            ColorRGBA reading = getColorReading.get();
            float minDistance = Float.MAX_VALUE;
            BallState predictedState = BallState.NONE;
            for (ColorRGBA key : ballStateMapping.keySet()) {
                float distance = key.distanceSquared(reading);
                if (distance < minDistance) {
                    minDistance = distance;
                    predictedState = ballStateMapping.get(key);
                }
            }

            return predictedState;

        }

        @Override
        public IAction<Telemetry> getTelemetryAction() {
            return telemetryAction.get();
        }
    }
    public static class TargetPositionCommander implements WithTelemetry.IWithTelemetry {
        static class BallSlot {
            public float pos;
            public BallState state;

            public BallSlot(float _pos, BallState _state) {
                pos = _pos;
                state = _state;
            }
        }
        static class IndexerSlots {
            BallSlot slot1, slot2, slot3;

            public boolean hasState(BallState state) {
                return slot1.state == state ||
                        slot2.state == state ||
                        slot3.state == state;
            }
            public Optional<BallSlot> getNext(BallState state) {
                if (slot1.state == state) {
                    return Optional.of(slot1);
                }
                else if (slot2.state == state) {
                    return Optional.of(slot2);
                }
                else if (slot3.state == state) {
                    return Optional.of(slot3);
                }

                return Optional.empty();

            }

            public IndexerSlots(BallSlot _slot1, BallSlot _slot2, BallSlot _slot3) {
                slot1 = _slot1;
                slot2 = _slot2;
                slot3 = _slot3;
            }
        }

        IndexerSlots slots;
        BallSlot targetSlot;


        LazyInit<IAction<BallState>> goToBallStateAction;

        LazyInit<IAction<Telemetry>> telemetryAction;

        public IAction<BallState> goToBallState() {return goToBallStateAction.get();}
        public float getTargetAngle() {return targetSlot.pos;}

        public void setTargetSlotState(BallState newState) {
            targetSlot.state = newState;
        }


        private BallSlot getNearestBallSlot(float pos) {
            float minDist = Math.abs(slots.slot1.pos - pos);
            BallSlot minSlot = slots.slot1;

            if (Math.abs(slots.slot2.pos - pos) < minDist) {
                minDist = Math.abs(slots.slot2.pos - pos);
                minSlot = slots.slot2;
            }

            if (Math.abs(slots.slot3.pos - pos) < minDist) {
                minDist = Math.abs(slots.slot3.pos - pos);
                minSlot = slots.slot3;
            }

            return minSlot;
        }

        public BallState getNearestBallPosState(float pos) {

            return getNearestBallSlot(pos).state;
        }
        public void setNearestBallState(float pos, BallState state) {
            getNearestBallSlot(pos).state = state;
        }

        public void setNearestBallStateGivenBallWithinThresh(float pos, BallState state, float thresh) {
            if (state == BallState.NONE || Math.abs(pos - getNearestBallSlot(pos).pos) > thresh) return;

            setNearestBallState(pos, state);
        }

        public float getPosFromIndex(int idx) {
            if (idx == 0) {
                return slots.slot1.pos;
            }
            if (idx == 1) {
                return slots.slot2.pos;
            }
            return slots.slot3.pos;

        }

        public TargetPositionCommander() {
            slots = new IndexerSlots(
                    new BallSlot(0, BallState.NONE),
                    new BallSlot(/*2f/3f * (float)Math.PI*/181, BallState.NONE),
                    new BallSlot(/*4f/3f * (float)Math.PI*/-181, BallState.NONE)
            );
            targetSlot = slots.slot1;

            goToBallStateAction = new LazyInit<>( () ->
                    IAction.From.loop((r, s) -> {
                        if (slots.hasState(s) ) {
                            slots.getNext(s).ifPresent(ballSlot -> targetSlot = ballSlot);
                        }
                    })
            );

            telemetryAction = new LazyInit<>(() -> WithTelemetry.fromLambda(
                    () -> "Spindexer Target Position Commander",
                    telemetry -> {
                        telemetry.addData("Target pos", targetSlot.pos);
                        telemetry.addData("Target color", targetSlot.state.toString());
                        telemetry.addData("Slot colors",
                                slots.slot1.state.toString() + ", "
                                + slots.slot2.state.toString() + ", "
                                + slots.slot3.state.toString()
                        );

                    }));
        }

        @Override
        public IAction<Telemetry> getTelemetryAction() {
            return telemetryAction.get();
        }

        public void setSlots(BallState init, BallState clock, BallState counter) {
            slots.slot1.state = init;
            slots.slot2.state = clock;
            slots.slot3.state = counter;
        }
    }


    public static class SpindexerPositionController implements WithTelemetry.IWithTelemetry{
        PIDF.Controller posPID;
        DcMotorWrapper motor;
        Supplier<Float> getTargetAngle;

        LazyInit<IAction<Float>> powerMotorAction;
        public IAction<Float> powerMotor() {return powerMotorAction.get();}

        LazyInit<IAction<Telemetry>> telemetryAction;

        public SpindexerPositionController(
                PIDF.Weights _coeff,
                DcMotorWrapper _motor,
                Supplier<Float> _getTargetAngle
        ) {
            posPID = new PIDF.Controller(_coeff);
            motor = _motor;
            getTargetAngle = _getTargetAngle;

            powerMotorAction = new LazyInit<>(() -> IAction.From.loop(
                    (r, dt) ->
                            motor.setPower(
                                Util.clamp(posPID.loop(
                                    motor.getPosition(),
                                    getTargetAngle.get(),
                                    dt
                                ), -1f, 1f) * 0.2f
                            )
            ));

            telemetryAction = new LazyInit<>(() -> WithTelemetry.fromLambda(
                    () -> "Spindexer Position Controller",
                    telemetry -> {
                        telemetry.addData("Current angle (rad)", motor.getAngle());
                        telemetry.addData("Current angular velocity (rad/s)",
                                motor.getAngularVelocity());
                        telemetry.addData("Target angle (rad)", getTargetAngle.get());
                        telemetry.addData("Motor pos (ticks)", motor.getPosition());
                        telemetry.addData("Unnormalized motor angle (rad)", motor.getPosition() / motor.getSpec().encoderResolution / 0.13f);
                    }
            ));


        }

        public float getMotorPos() {
            return motor.getPosition();
        }

        @Override
        public IAction<Telemetry> getTelemetryAction() {
            return telemetryAction.get();
        }
    }
    public static class BallStateUpdater {
        Supplier<Float> getPos;
    }

}