package com.example;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.file.StandardOpenOption;

public final class RawSdfVolume implements Sdf {
    private final float[] samples;
    private final int sizeX;
    private final int sizeY;
    private final int sizeZ;
    private final double domainMin;
    private final double domainMax;
    private final double invDomainSize;

    private RawSdfVolume(float[] samples, int sizeX, int sizeY, int sizeZ, double domainMin, double domainMax) {
        this.samples = samples;
        this.sizeX = sizeX;
        this.sizeY = sizeY;
        this.sizeZ = sizeZ;
        this.domainMin = domainMin;
        this.domainMax = domainMax;
        this.invDomainSize = 1.0 / (domainMax - domainMin);
    }

    public static RawSdfVolume load(File file, int sizeX, int sizeY, int sizeZ, double domainMin, double domainMax) {
        validateDimensions(sizeX, sizeY, sizeZ);
        if (file == null || !file.isFile()) {
            throw new IllegalArgumentException("Raw SDF file does not exist.");
        }

        int sampleCount = checkedSampleCount(sizeX, sizeY, sizeZ);
        long expectedBytes = (long) sampleCount * Float.BYTES;
        if (expectedBytes > Integer.MAX_VALUE) {
            throw new IllegalArgumentException("Raw SDF file is too large for this demo loader.");
        }
        long actualBytes = file.length();
        if (actualBytes != expectedBytes) {
            throw new IllegalArgumentException(String.format(
                    "Raw SDF file has %,d bytes; expected %,d bytes for %dx%dx%d float32 samples.",
                    actualBytes,
                    expectedBytes,
                    sizeX,
                    sizeY,
                    sizeZ));
        }

        float[] samples = new float[sampleCount];
        ByteBuffer buffer = ByteBuffer.allocate((int) expectedBytes);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        try (FileChannel channel = FileChannel.open(file.toPath(), StandardOpenOption.READ)) {
            while (buffer.hasRemaining() && channel.read(buffer) != -1) {
                // Keep reading until the exact float32 payload is available.
            }
        } catch (IOException exception) {
            throw new IllegalArgumentException("Could not read raw SDF file: " + exception.getMessage(), exception);
        }

        buffer.flip();
        for (int i = 0; i < samples.length; i++) {
            samples[i] = buffer.getFloat();
        }
        return new RawSdfVolume(samples, sizeX, sizeY, sizeZ, domainMin, domainMax);
    }

    public static RawSdfVolume sample(Sdf sdf, int sizeX, int sizeY, int sizeZ, double domainMin, double domainMax) {
        validateDimensions(sizeX, sizeY, sizeZ);
        int sampleCount = checkedSampleCount(sizeX, sizeY, sizeZ);
        float[] samples = new float[sampleCount];

        /*
         * This creates an in-memory raw-style SDF volume from an analytic SDF.
         * Later eval() calls only see the voxel grid and trilinear interpolation,
         * matching the sampling behavior used for external raw SDF files.
         */
        for (int z = 0; z < sizeZ; z++) {
            double pz = gridCoordinate(z, sizeZ, domainMin, domainMax);
            for (int y = 0; y < sizeY; y++) {
                double py = gridCoordinate(y, sizeY, domainMin, domainMax);
                for (int x = 0; x < sizeX; x++) {
                    double px = gridCoordinate(x, sizeX, domainMin, domainMax);
                    samples[x + sizeX * (y + sizeY * z)] = (float) sdf.eval(new Vec3(px, py, pz));
                }
            }
        }

        return new RawSdfVolume(samples, sizeX, sizeY, sizeZ, domainMin, domainMax);
    }

    @Override
    public double eval(Vec3 p) {
        double gx = toGridCoordinate(p.x, sizeX);
        double gy = toGridCoordinate(p.y, sizeY);
        double gz = toGridCoordinate(p.z, sizeZ);

        int x0 = (int) Math.floor(gx);
        int y0 = (int) Math.floor(gy);
        int z0 = (int) Math.floor(gz);
        int x1 = Math.min(x0 + 1, sizeX - 1);
        int y1 = Math.min(y0 + 1, sizeY - 1);
        int z1 = Math.min(z0 + 1, sizeZ - 1);
        double tx = gx - x0;
        double ty = gy - y0;
        double tz = gz - z0;

        double c00 = lerp(sample(x0, y0, z0), sample(x1, y0, z0), tx);
        double c10 = lerp(sample(x0, y1, z0), sample(x1, y1, z0), tx);
        double c01 = lerp(sample(x0, y0, z1), sample(x1, y0, z1), tx);
        double c11 = lerp(sample(x0, y1, z1), sample(x1, y1, z1), tx);
        double c0 = lerp(c00, c10, ty);
        double c1 = lerp(c01, c11, ty);
        return lerp(c0, c1, tz);
    }

    public int getSizeX() {
        return sizeX;
    }

    public int getSizeY() {
        return sizeY;
    }

    public int getSizeZ() {
        return sizeZ;
    }

    private double toGridCoordinate(double value, int size) {
        double t = (value - domainMin) * invDomainSize;
        double grid = t * (size - 1);
        return clamp(grid, 0.0, size - 1);
    }

    private double sample(int x, int y, int z) {
        // Raw volume order is x-fastest, then y, then z.
        return samples[x + sizeX * (y + sizeY * z)];
    }

    private static void validateDimensions(int sizeX, int sizeY, int sizeZ) {
        if (sizeX < 2 || sizeY < 2 || sizeZ < 2) {
            throw new IllegalArgumentException("Raw SDF dimensions must all be at least 2.");
        }
    }

    private static int checkedSampleCount(int sizeX, int sizeY, int sizeZ) {
        long sampleCount = (long) sizeX * sizeY * sizeZ;
        if (sampleCount > Integer.MAX_VALUE) {
            throw new IllegalArgumentException("Raw SDF volume has too many samples for this demo loader.");
        }
        return (int) sampleCount;
    }

    private static double gridCoordinate(int index, int size, double domainMin, double domainMax) {
        return domainMin + (domainMax - domainMin) * index / (size - 1);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
