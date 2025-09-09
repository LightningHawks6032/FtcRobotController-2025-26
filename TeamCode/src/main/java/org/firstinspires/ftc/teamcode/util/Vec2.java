package org.firstinspires.ftc.teamcode.util;

public class Vec2 {
    public float x;
    public float y;
    public Vec2(float _x, float _y) {
        x = _x;
        y = _y;
    }

    public Vec2 add(Vec2 other) {
        return new Vec2(x + other.x, y + other.y);
    }
    public Vec2 sub(Vec2 other) {
        return new Vec2(x - other.x, y - other.y);
    }
    public Vec2 scale(float scalar) {
        return new Vec2(x * scalar, y * scalar);
    }
    public float dot(Vec2 other) {
        return x * other.x + y * other.y;
    }
    public float cross(Vec2 other) {
        return x * other.y - y * other.x;
    }
    public float mag() {
        return (float) Math.sqrt(x * x + y * y);
    }
    public Vec2 norm() {
        float mag = mag();
        return new Vec2(x / mag, y / mag);
    }
    public boolean nonzero() {
        return x!=0||y!=0;
    }
}
