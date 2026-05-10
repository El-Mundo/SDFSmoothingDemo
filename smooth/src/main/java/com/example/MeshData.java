package com.example;

import java.util.Collections;
import java.util.List;

public final class MeshData {
    private final List<Vec3> vertices;
    private final List<int[]> triangles;
    private final List<Vec3> faceNormals;
    private final int activeCellCount;
    private final long buildTimeMillis;
    private final int qefSolvedCount;
    private final int massFallbackCount;
    private final int hardClampCount;
    private final int slackClampCount;
    private final double averageQefError;
    private final int projectedVertexCount;
    private final double averageAbsSdfBeforeProjection;
    private final double averageAbsSdfAfterProjection;

    public MeshData(List<Vec3> vertices, List<int[]> triangles, int activeCellCount, long buildTimeMillis) {
        this(vertices, triangles, activeCellCount, buildTimeMillis, 0, 0, 0, 0, 0.0, 0, 0.0, 0.0);
    }

    public MeshData(
            List<Vec3> vertices,
            List<int[]> triangles,
            int activeCellCount,
            long buildTimeMillis,
            int qefSolvedCount,
            int massFallbackCount,
            int hardClampCount,
            int slackClampCount,
            double averageQefError,
            int projectedVertexCount,
            double averageAbsSdfBeforeProjection,
            double averageAbsSdfAfterProjection) {
        this.vertices = Collections.unmodifiableList(vertices);
        this.triangles = Collections.unmodifiableList(triangles);
        this.faceNormals = Collections.unmodifiableList(computeFaceNormals(vertices, triangles));
        this.activeCellCount = activeCellCount;
        this.buildTimeMillis = buildTimeMillis;
        this.qefSolvedCount = qefSolvedCount;
        this.massFallbackCount = massFallbackCount;
        this.hardClampCount = hardClampCount;
        this.slackClampCount = slackClampCount;
        this.averageQefError = averageQefError;
        this.projectedVertexCount = projectedVertexCount;
        this.averageAbsSdfBeforeProjection = averageAbsSdfBeforeProjection;
        this.averageAbsSdfAfterProjection = averageAbsSdfAfterProjection;
    }

    public List<Vec3> getVertices() {
        return vertices;
    }

    public List<int[]> getTriangles() {
        return triangles;
    }

    public List<Vec3> getFaceNormals() {
        return faceNormals;
    }

    public int getActiveCellCount() {
        return activeCellCount;
    }

    public long getBuildTimeMillis() {
        return buildTimeMillis;
    }

    public int getQefSolvedCount() {
        return qefSolvedCount;
    }

    public int getMassFallbackCount() {
        return massFallbackCount;
    }

    public int getHardClampCount() {
        return hardClampCount;
    }

    public int getSlackClampCount() {
        return slackClampCount;
    }

    public double getAverageQefError() {
        return averageQefError;
    }

    public int getProjectedVertexCount() {
        return projectedVertexCount;
    }

    public double getAverageAbsSdfBeforeProjection() {
        return averageAbsSdfBeforeProjection;
    }

    public double getAverageAbsSdfAfterProjection() {
        return averageAbsSdfAfterProjection;
    }

    private static List<Vec3> computeFaceNormals(List<Vec3> vertices, List<int[]> triangles) {
        List<Vec3> normals = new java.util.ArrayList<>();
        for (int[] triangle : triangles) {
            Vec3 a = vertices.get(triangle[0]);
            Vec3 b = vertices.get(triangle[1]);
            Vec3 c = vertices.get(triangle[2]);
            Vec3 cross = b.subtract(a).cross(c.subtract(a));
            if (cross.lengthSquared() < 1.0e-20) {
                normals.add(new Vec3(0.0, 1.0, 0.0));
            } else {
                normals.add(cross.normalize());
            }
        }
        return normals;
    }
}
