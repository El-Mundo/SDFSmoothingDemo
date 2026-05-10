package com.example;

public final class SdfLibrary {
    private SdfLibrary() {
    }

    public static Sdf sphere(double radius) {
        return p -> p.length() - radius;
    }

    /*
     * Torus is mainly for checking smooth Hermite normals, curved feature
     * reconstruction, and the optional post-QEF surface projection polish.
     */
    public static Sdf torus(double majorRadius, double minorRadius) {
        return p -> {
            double qx = Math.sqrt(p.x * p.x + p.z * p.z) - majorRadius;
            return Math.sqrt(qx * qx + p.y * p.y) - minorRadius;
        };
    }

    public static Sdf box(Vec3 halfExtents) {
        return p -> {
            Vec3 q = p.abs().subtract(halfExtents);
            double outside = q.max(0.0).length();
            double inside = Math.min(q.maxComponent(), 0.0);
            return outside + inside;
        };
    }

    public static Sdf rotatedBox(Vec3 halfExtents, double angleYRadians) {
        return rotatedBox(halfExtents, angleYRadians, 0.0, 0.0);
    }

    public static Sdf rotatedBox(Vec3 halfExtents, double angleYRadians, double angleXRadians) {
        return rotatedBox(halfExtents, angleYRadians, angleXRadians, 0.0);
    }

    public static Sdf rotatedBox(Vec3 halfExtents, double angleYRadians, double angleXRadians, double angleZRadians) {
        Sdf box = box(halfExtents);
        return p -> box.eval(toLocalSpace(p, angleYRadians, angleXRadians, angleZRadians));
    }

    public static Sdf pyramid(double height, double halfBase) {
        double h = Math.max(height, 1.0e-9);
        double sideSlope = halfBase / h;
        double sideNormalizer = Math.sqrt(1.0 + sideSlope * sideSlope);
        double halfHeight = h * 0.5;

        return p -> {
            double baseDistance = -p.y - halfHeight;
            double positiveXDistance = (p.x + sideSlope * p.y - halfBase * 0.5) / sideNormalizer;
            double negativeXDistance = (-p.x + sideSlope * p.y - halfBase * 0.5) / sideNormalizer;
            double positiveZDistance = (p.z + sideSlope * p.y - halfBase * 0.5) / sideNormalizer;
            double negativeZDistance = (-p.z + sideSlope * p.y - halfBase * 0.5) / sideNormalizer;

            return Math.max(baseDistance, Math.max(
                    Math.max(positiveXDistance, negativeXDistance),
                    Math.max(positiveZDistance, negativeZDistance)));
        };
    }

    public static Sdf union(Sdf a, Sdf b) {
        return p -> Math.min(a.eval(p), b.eval(p));
    }

    public static Sdf intersection(Sdf a, Sdf b) {
        return p -> Math.max(a.eval(p), b.eval(p));
    }

    public static Sdf subtract(Sdf a, Sdf b) {
        return p -> Math.max(a.eval(p), -b.eval(p));
    }

    /*
     * RotatedCube is meant to reproduce the Reddit VoxelGameDev issue where
     * grid-edge-based methods wobble or lose sharp features when a sharp object
     * is badly aligned to the grid.
     */
    public static Sdf rotatedCube(double angleRadians) {
        return rotatedBox(new Vec3(0.65, 0.65, 0.65), angleRadians, angleRadians);
    }

    /*
     * ThinRotatedBox targets the same grid-alignment problem as RotatedCube,
     * with extra pressure from thin features that can be missed by grid edges.
     */
    public static Sdf thinRotatedBox(double angleRadians) {
        return rotatedBox(new Vec3(0.85, 0.12, 0.55), angleRadians, 0.0, Math.toRadians(17.0));
    }

    public static Sdf boxMinusSphere() {
        /*
         * BoxMinusSphere tests non-smooth CSG gradients where the subtraction
         * seam crosses the cube faces: subtract uses max(a, -b).
         */
        Sdf cube = box(new Vec3(0.78, 0.78, 0.78));
        Sdf sphere = sphere(0.9);
        return subtract(cube, sphere);
    }

    public static Sdf doubleTorusOrUnion() {
        /*
         * Torus is mainly useful for checking smooth Hermite normals and the
         * optional SDF projection step; a double union also exercises topology.
         */
        Sdf torus = torus(0.38, 0.16);
        Vec3 left = new Vec3(-0.38, 0.0, 0.0);
        Vec3 right = new Vec3(0.38, 0.0, 0.0);
        Sdf torusA = p -> torus.eval(p.subtract(left));
        Sdf torusB = p -> torus.eval(p.subtract(right));
        return union(torusA, torusB);
    }

    public static Sdf pyramidOnCube() {
        /*
         * PyramidOnCube tests non-smooth CSG gradients at the crease between
         * the cube base and the pyramid top.
         */
        Sdf cube = box(new Vec3(0.65, 0.35, 0.65));
        Sdf pyramid = pyramid(0.9, 0.65);
        Vec3 cubeOffset = new Vec3(0.0, -0.4, 0.0);
        Vec3 pyramidOffset = new Vec3(0.0, 0.4, 0.0);
        Sdf cubeBase = p -> cube.eval(p.subtract(cubeOffset));
        Sdf pyramidTop = p -> pyramid.eval(p.subtract(pyramidOffset));
        return union(cubeBase, pyramidTop);
    }

    public static Sdf smoothUnion(Sdf a, Sdf b, double k) {
        double smoothing = Math.max(k, 1.0e-9);
        return p -> {
            double da = a.eval(p);
            double db = b.eval(p);
            double h = clamp(0.5 + 0.5 * (db - da) / smoothing, 0.0, 1.0);
            return lerp(db, da, h) - smoothing * h * (1.0 - h);
        };
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static Vec3 toLocalSpace(Vec3 p, double angleYRadians, double angleXRadians, double angleZRadians) {
        // Rotated SDFs transform the sample point by inverse rotations, then evaluate the unrotated primitive.
        Vec3 q = rotateY(p, -angleYRadians);
        q = rotateX(q, -angleXRadians);
        q = rotateZ(q, -angleZRadians);
        return q;
    }

    private static Vec3 rotateX(Vec3 p, double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);
        return new Vec3(
                p.x,
                p.y * cos - p.z * sin,
                p.y * sin + p.z * cos);
    }

    private static Vec3 rotateY(Vec3 p, double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);
        return new Vec3(
                p.x * cos + p.z * sin,
                p.y,
                -p.x * sin + p.z * cos);
    }

    private static Vec3 rotateZ(Vec3 p, double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);
        return new Vec3(
                p.x * cos - p.y * sin,
                p.x * sin + p.y * cos,
                p.z);
    }
}
