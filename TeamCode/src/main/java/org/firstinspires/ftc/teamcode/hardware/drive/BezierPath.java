package org.firstinspires.ftc.teamcode.hardware.drive;

import org.firstinspires.ftc.teamcode.util.Vec2;

public class BezierPath {
    public static class Cubic implements IPath<Vec2> {

        Vec2 p0,p1,p2,p3;
        float duration = 1;

        @Override
        public float getDuration() {
            return duration;
        }

        @Override
        public PathPoint<Vec2> getPoint(float elapsed) {
            float t = elapsed/duration;
            float oneMinusT = 1-t;
            /// guh
            return new PathPoint<>(
                    p3.scale(t*t*t).add(
                            (p2.scale(3*t*t)
                                    .add(
                                            (p1.scale(3*t).add(
                                                    p0.scale(oneMinusT)
                                            )).scale(oneMinusT)
                                    )).scale(oneMinusT)
                    ),
                    (p3.sub(p2)).scale(3*t*t).add(
                            (p2.sub(p1)).scale(t*oneMinusT*6).add(
                                    (p1.sub(p0)).scale(oneMinusT * oneMinusT * 3)
                            )
                    ),
                    (
                            (p2.sub(p1.scale(2)).add(p0).scale(oneMinusT)).add(
                                    p3.sub(p2.scale(2)).add(p1).scale(t)
                            )
                            ).scale(6)

            );
        }

        public Cubic(Vec2 _p0, Vec2 _p1, Vec2 _p2, Vec2 _p3, float _duration) {
            p0=_p0;
            p1=_p1;
            p2=_p2;
            p3=_p3;

            duration = _duration;
        }
    }
}
