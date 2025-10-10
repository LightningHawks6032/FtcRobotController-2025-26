    package org.firstinspires.ftc.teamcode.util;

    import androidx.annotation.NonNull;

    public class Vec2Rot {
        public float x;
        public float y;
        public float r;
        public Vec2Rot(float _x, float _y, float _r) {
            x = _x;
            y = _y;
            r = _r;
        }
        public Vec2Rot(@NonNull Vec2 p, float _r) {
            x = p.x;
            y = p.y;
            r = _r;
        }

        public Vec2Rot componentwiseScl(float scl) {
            return new Vec2Rot(x * scl, y * scl, r * scl);
        }

        public Vec2Rot componentwiseAdd(Vec2Rot v) {
            return new Vec2Rot(x + v.x, y + v.y, r + v.r);
        }

        public Vec2Rot componentwiseSub(Vec2Rot v) {
            return new Vec2Rot(x - v.x, y - v.y, r - v.r);
        }

        public Vec2 asVec2() {
            return new Vec2(x, y);
        }
        static Vec2Rot _zero = new Vec2Rot(0,0,0);
        public static Vec2Rot zero() {return _zero;}

        @NonNull
        public String toString() {
            return "(" + x + ", " + y + ", " + r + ")";
        }
    }
