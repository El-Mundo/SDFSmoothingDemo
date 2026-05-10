package com.example;

public final class Vec3 {
    public final double x;
    public final double y;
    public final double z;

    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vec3 add(Vec3 other) {
        return new Vec3(x + other.x, y + other.y, z + other.z);
    }

    public Vec3 subtract(Vec3 other) {
        return new Vec3(x - other.x, y - other.y, z - other.z);
    }

    public Vec3 multiply(double scalar) {
        return new Vec3(x * scalar, y * scalar, z * scalar);
    }

    public Vec3 divide(double scalar) {
        return new Vec3(x / scalar, y / scalar, z / scalar);
    }

    public double dot(Vec3 other) {
        return x * other.x + y * other.y + z * other.z;
    }

    public Vec3 cross(Vec3 other) {
        return new Vec3(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x);
    }

    public double lengthSquared() {
        return dot(this);
    }

    public double length() {
        return Math.sqrt(lengthSquared());
    }

    public Vec3 normalize() {
        double len = length();
        if (len == 0.0) {
            return new Vec3(0.0, 0.0, 0.0);
        }
        return divide(len);
    }

    public Vec3 abs() {
        return new Vec3(Math.abs(x), Math.abs(y), Math.abs(z));
    }

    public Vec3 max(double value) {
        return new Vec3(Math.max(x, value), Math.max(y, value), Math.max(z, value));
    }

    public Vec3 clamp(Vec3 min, Vec3 max) {
        return new Vec3(
                clamp(x, min.x, max.x),
                clamp(y, min.y, max.y),
                clamp(z, min.z, max.z));
    }

    public Vec3 lerp(Vec3 other, double t) {
        return new Vec3(
                x + (other.x - x) * t,
                y + (other.y - y) * t,
                z + (other.z - z) * t);
    }

    public double maxComponent() {
        return Math.max(x, Math.max(y, z));
    }

    public boolean isFinite() {
        return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
    }

    public static Vec3 zero() {
        return new Vec3(0.0, 0.0, 0.0);
    }

    @Override
    public String toString() {
        return "Vec3{" + "x=" + x + ", y=" + y + ", z=" + z + '}';
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
