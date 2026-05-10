package com.example;

import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class MeshBuilder {
    private MeshBuilder() {
    }

    public static TriangleMesh build(List<Vec3> vertices, int[] triangleIndices) {
        if (triangleIndices.length % 3 != 0) {
            throw new IllegalArgumentException("Triangle index array length must be divisible by 3.");
        }

        float[] points = new float[vertices.size() * 3];
        for (int i = 0; i < vertices.size(); i++) {
            Vec3 v = vertices.get(i);
            points[i * 3] = (float) v.x;
            points[i * 3 + 1] = (float) v.y;
            points[i * 3 + 2] = (float) v.z;
        }

        int[] faces = new int[triangleIndices.length * 2];
        for (int i = 0; i < triangleIndices.length; i++) {
            int vertexIndex = triangleIndices[i];
            if (vertexIndex < 0 || vertexIndex >= vertices.size()) {
                throw new IllegalArgumentException("Triangle index out of range: " + vertexIndex);
            }
            faces[i * 2] = vertexIndex;
            faces[i * 2 + 1] = 0;
        }

        TriangleMesh mesh = new TriangleMesh();
        mesh.getPoints().setAll(points);
        mesh.getTexCoords().setAll(0.0f, 0.0f);
        mesh.getFaces().setAll(faces);
        return mesh;
    }

    public static TriangleMesh build(MeshData meshData) {
        return build(meshData.getVertices(), meshData.getTriangles());
    }

    public static TriangleMesh build(MeshData meshData, Sdf sdf, NormalMode normalMode, double autoSmoothAngleDegrees) {
        return build(meshData, sdf, normalMode, autoSmoothAngleDegrees, UvGenerator.Mode.BOX_PROJECTED);
    }

    public static TriangleMesh build(
            MeshData meshData,
            Sdf sdf,
            NormalMode normalMode,
            double autoSmoothAngleDegrees,
            UvGenerator.Mode uvMode) {
        UvGenerator.Result uvResult = UvGenerator.generate(meshData, uvMode);
        if (normalMode == NormalMode.FLAT) {
            return buildFlat(meshData.getVertices(), meshData.getTriangles(), uvResult);
        }
        if (normalMode == NormalMode.SDF_GRADIENT) {
            return buildWithSdfGradientNormals(meshData.getVertices(), meshData.getTriangles(), sdf, uvResult);
        }
        return buildWithAutoSmoothNormals(meshData.getVertices(), meshData.getTriangles(), autoSmoothAngleDegrees, uvResult);
    }

    public static TriangleMesh build(List<Vec3> vertices, List<int[]> triangles) {
        List<Integer> indices = new ArrayList<>();
        for (int[] triangle : triangles) {
            if (triangle.length != 3) {
                throw new IllegalArgumentException("Each triangle must contain exactly 3 vertex indices.");
            }
            indices.add(triangle[0]);
            indices.add(triangle[1]);
            indices.add(triangle[2]);
        }

        int[] triangleIndices = new int[indices.size()];
        for (int i = 0; i < indices.size(); i++) {
            triangleIndices[i] = indices.get(i);
        }
        return build(vertices, triangleIndices);
    }

    public static TriangleMesh placeholderCube(double sideLength) {
        double h = sideLength * 0.5;
        List<Vec3> vertices = Arrays.asList(
                new Vec3(-h, -h, -h),
                new Vec3(h, -h, -h),
                new Vec3(h, h, -h),
                new Vec3(-h, h, -h),
                new Vec3(-h, -h, h),
                new Vec3(h, -h, h),
                new Vec3(h, h, h),
                new Vec3(-h, h, h));

        int[] triangles = {
                0, 2, 1, 0, 3, 2,
                4, 5, 6, 4, 6, 7,
                0, 1, 5, 0, 5, 4,
                3, 6, 2, 3, 7, 6,
                1, 2, 6, 1, 6, 5,
                0, 4, 7, 0, 7, 3
        };

        return build(vertices, triangles);
    }

    private static TriangleMesh buildFlat(List<Vec3> vertices, List<int[]> triangles, UvGenerator.Result uvResult) {
        List<Vec3> flatVertices = new ArrayList<>();
        List<Vec3> flatNormals = new ArrayList<>();
        int[] faces = new int[triangles.size() * 9];
        int faceOffset = 0;

        for (int faceIndex = 0; faceIndex < triangles.size(); faceIndex++) {
            int[] triangle = triangles.get(faceIndex);
            Vec3 a = vertices.get(triangle[0]);
            Vec3 b = vertices.get(triangle[1]);
            Vec3 c = vertices.get(triangle[2]);
            Vec3 normal = faceNormal(a, b, c);
            int normalIndex = flatNormals.size();
            flatNormals.add(normal);

            for (int corner = 0; corner < 3; corner++) {
                int pointIndex = flatVertices.size();
                flatVertices.add(vertices.get(triangle[corner]));
                faces[faceOffset++] = pointIndex;
                faces[faceOffset++] = normalIndex;
                faces[faceOffset++] = uvResult.getTexCoordIndex(faceIndex, corner);
            }
        }

        return buildWithNormals(flatVertices, flatNormals, uvResult.getTexCoords(), faces);
    }

    private static TriangleMesh buildWithSdfGradientNormals(
            List<Vec3> vertices,
            List<int[]> triangles,
            Sdf sdf,
            UvGenerator.Result uvResult) {
        List<Vec3> normals = new ArrayList<>();
        int[] faces = new int[triangles.size() * 9];
        int faceOffset = 0;
        double eps = 1.0e-4;

        /*
         * SDF_GRADIENT normals are the basic Hermite/SDF normal approach:
         * estimate the local SDF gradient at the rendered vertex position.
         */
        for (int faceIndex = 0; faceIndex < triangles.size(); faceIndex++) {
            int[] triangle = triangles.get(faceIndex);
            Vec3 faceNormal = faceNormal(
                    vertices.get(triangle[0]),
                    vertices.get(triangle[1]),
                    vertices.get(triangle[2]));

            for (int corner = 0; corner < 3; corner++) {
                int vertexIndex = triangle[corner];
                Vec3 normal = estimateSdfNormal(sdf, vertices.get(vertexIndex), eps);
                if (normal.lengthSquared() < 1.0e-20) {
                    normal = faceNormal;
                }

                int normalIndex = normals.size();
                normals.add(normal);
                faces[faceOffset++] = vertexIndex;
                faces[faceOffset++] = normalIndex;
                faces[faceOffset++] = uvResult.getTexCoordIndex(faceIndex, corner);
            }
        }

        return buildWithNormals(vertices, normals, uvResult.getTexCoords(), faces);
    }

    private static TriangleMesh buildWithAutoSmoothNormals(
            List<Vec3> vertices,
            List<int[]> triangles,
            double autoSmoothAngleDegrees,
            UvGenerator.Result uvResult) {
        List<FaceInfo> faceInfos = new ArrayList<>();
        Map<PositionKey, List<FaceInfo>> facesByPosition = new HashMap<>();

        for (int faceIndex = 0; faceIndex < triangles.size(); faceIndex++) {
            int[] triangle = triangles.get(faceIndex);
            Vec3 a = vertices.get(triangle[0]);
            Vec3 b = vertices.get(triangle[1]);
            Vec3 c = vertices.get(triangle[2]);
            Vec3 cross = b.subtract(a).cross(c.subtract(a));
            double doubleArea = cross.length();
            Vec3 normal = doubleArea < 1.0e-20 ? new Vec3(0.0, 1.0, 0.0) : cross.divide(doubleArea);
            FaceInfo faceInfo = new FaceInfo(faceIndex, normal, doubleArea);
            faceInfos.add(faceInfo);

            for (int vertexIndex : triangle) {
                PositionKey key = PositionKey.from(vertices.get(vertexIndex));
                facesByPosition.computeIfAbsent(key, ignored -> new ArrayList<>()).add(faceInfo);
            }
        }

        List<Vec3> normals = new ArrayList<>();
        int[] faces = new int[triangles.size() * 9];
        int faceOffset = 0;
        double cosThreshold = Math.cos(Math.toRadians(autoSmoothAngleDegrees));

        /*
         * AUTO_SMOOTH follows the idea from the realtime GPU autosmoothing
         * post: group nearby/identical vertices, then smooth by face-normal
         * angle threshold. This does not require original mesh adjacency.
         */
        for (int faceIndex = 0; faceIndex < triangles.size(); faceIndex++) {
            int[] triangle = triangles.get(faceIndex);
            FaceInfo currentFace = faceInfos.get(faceIndex);

            for (int corner = 0; corner < 3; corner++) {
                int vertexIndex = triangle[corner];
                PositionKey key = PositionKey.from(vertices.get(vertexIndex));
                Vec3 normal = averageNearbyFaceNormals(facesByPosition.get(key), currentFace, cosThreshold);
                int normalIndex = normals.size();
                normals.add(normal);
                faces[faceOffset++] = vertexIndex;
                faces[faceOffset++] = normalIndex;
                faces[faceOffset++] = uvResult.getTexCoordIndex(faceIndex, corner);
            }
        }

        return buildWithNormals(vertices, normals, uvResult.getTexCoords(), faces);
    }

    private static TriangleMesh buildWithNormals(List<Vec3> vertices, List<Vec3> normals, List<Vec2> texCoords, int[] faces) {
        TriangleMesh mesh = new TriangleMesh(VertexFormat.POINT_NORMAL_TEXCOORD);
        mesh.getPoints().setAll(toFloatArray(vertices));
        mesh.getNormals().setAll(toFloatArray(normals));
        mesh.getTexCoords().setAll(toTexFloatArray(texCoords));
        mesh.getFaces().setAll(faces);
        return mesh;
    }

    private static Vec3 averageNearbyFaceNormals(List<FaceInfo> adjacentFaces, FaceInfo currentFace, double cosThreshold) {
        Vec3 sum = Vec3.zero();
        if (adjacentFaces != null) {
            for (FaceInfo adjacent : adjacentFaces) {
                if (currentFace.normal.dot(adjacent.normal) >= cosThreshold) {
                    sum = sum.add(adjacent.normal.multiply(adjacent.doubleArea));
                }
            }
        }

        if (sum.lengthSquared() < 1.0e-20) {
            return currentFace.normal;
        }
        return sum.normalize();
    }

    private static Vec3 estimateSdfNormal(Sdf sdf, Vec3 p, double eps) {
        double nx = sdf.eval(new Vec3(p.x + eps, p.y, p.z)) - sdf.eval(new Vec3(p.x - eps, p.y, p.z));
        double ny = sdf.eval(new Vec3(p.x, p.y + eps, p.z)) - sdf.eval(new Vec3(p.x, p.y - eps, p.z));
        double nz = sdf.eval(new Vec3(p.x, p.y, p.z + eps)) - sdf.eval(new Vec3(p.x, p.y, p.z - eps));
        return new Vec3(nx, ny, nz).normalize();
    }

    private static Vec3 faceNormal(Vec3 a, Vec3 b, Vec3 c) {
        Vec3 cross = b.subtract(a).cross(c.subtract(a));
        if (cross.lengthSquared() < 1.0e-20) {
            return new Vec3(0.0, 1.0, 0.0);
        }
        return cross.normalize();
    }

    private static float[] toFloatArray(List<Vec3> vectors) {
        float[] values = new float[vectors.size() * 3];
        for (int i = 0; i < vectors.size(); i++) {
            Vec3 v = vectors.get(i);
            values[i * 3] = (float) v.x;
            values[i * 3 + 1] = (float) v.y;
            values[i * 3 + 2] = (float) v.z;
        }
        return values;
    }

    private static float[] toTexFloatArray(List<Vec2> texCoords) {
        float[] values = new float[texCoords.size() * 2];
        for (int i = 0; i < texCoords.size(); i++) {
            Vec2 texCoord = texCoords.get(i);
            values[i * 2] = (float) texCoord.u;
            values[i * 2 + 1] = (float) texCoord.v;
        }
        return values;
    }

    private static final class FaceInfo {
        private final int faceIndex;
        private final Vec3 normal;
        private final double doubleArea;

        private FaceInfo(int faceIndex, Vec3 normal, double doubleArea) {
            this.faceIndex = faceIndex;
            this.normal = normal;
            this.doubleArea = doubleArea;
        }
    }

    private static final class PositionKey {
        private static final double SCALE = 1.0e6;

        private final long x;
        private final long y;
        private final long z;

        private PositionKey(long x, long y, long z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        private static PositionKey from(Vec3 p) {
            return new PositionKey(
                    Math.round(p.x * SCALE),
                    Math.round(p.y * SCALE),
                    Math.round(p.z * SCALE));
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof PositionKey)) {
                return false;
            }
            PositionKey other = (PositionKey) obj;
            return x == other.x && y == other.y && z == other.z;
        }

        @Override
        public int hashCode() {
            int result = Long.hashCode(x);
            result = 31 * result + Long.hashCode(y);
            result = 31 * result + Long.hashCode(z);
            return result;
        }
    }
}
