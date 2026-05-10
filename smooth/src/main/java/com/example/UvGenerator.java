package com.example;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class UvGenerator {
    private static final double UV_DOMAIN_MIN = DualContouring.DOMAIN_MIN;
    private static final double UV_DOMAIN_SIZE = DualContouring.DOMAIN_MAX - DualContouring.DOMAIN_MIN;

    private UvGenerator() {
    }

    public enum Mode {
        PLANAR_XY,
        PLANAR_XZ,
        PLANAR_YZ,
        BOX_PROJECTED,
        FACE_LOCAL,
        TRIPLANAR_READY
    }

    public static Result generate(MeshData meshData, Mode mode) {
        return generate(meshData.getVertices(), meshData.getTriangles(), meshData.getFaceNormals(), mode);
    }

    public static Result generate(List<Vec3> vertices, List<int[]> triangles, List<Vec3> faceNormals, Mode mode) {
        List<Vec2> texCoords = new ArrayList<>();
        List<Vec3> triplanarPositions = new ArrayList<>();
        List<Vec3> triplanarNormals = new ArrayList<>();
        int[] texCoordIndices = new int[triangles.size() * 3];
        int indexOffset = 0;

        for (int faceIndex = 0; faceIndex < triangles.size(); faceIndex++) {
            int[] triangle = triangles.get(faceIndex);
            Vec3 faceNormal = faceNormals.get(faceIndex);
            Vec2[] faceUvs = generateFaceUvs(vertices, triangle, faceNormal, mode);

            for (int corner = 0; corner < 3; corner++) {
                int texCoordIndex = texCoords.size();
                texCoords.add(faceUvs[corner]);
                texCoordIndices[indexOffset++] = texCoordIndex;

                if (mode == Mode.TRIPLANAR_READY) {
                    triplanarPositions.add(vertices.get(triangle[corner]));
                    triplanarNormals.add(faceNormal);
                }
            }
        }

        return new Result(texCoords, texCoordIndices, triplanarPositions, triplanarNormals);
    }

    private static Vec2[] generateFaceUvs(List<Vec3> vertices, int[] triangle, Vec3 faceNormal, Mode mode) {
        Vec3 a = vertices.get(triangle[0]);
        Vec3 b = vertices.get(triangle[1]);
        Vec3 c = vertices.get(triangle[2]);

        if (mode == Mode.FACE_LOCAL) {
            return faceLocalUvs(a, b, c);
        }
        if (mode == Mode.PLANAR_XY) {
            return new Vec2[] {xy(a), xy(b), xy(c)};
        }
        if (mode == Mode.PLANAR_XZ) {
            return new Vec2[] {xz(a), xz(b), xz(c)};
        }
        if (mode == Mode.PLANAR_YZ) {
            return new Vec2[] {yz(a), yz(b), yz(c)};
        }

        // JavaFX has no shader triplanar path here, so TRIPLANAR_READY is approximated by box projection.
        return boxProjectedUvs(a, b, c, faceNormal);
    }

    private static Vec2[] boxProjectedUvs(Vec3 a, Vec3 b, Vec3 c, Vec3 faceNormal) {
        Vec3 absNormal = faceNormal.abs();
        if (absNormal.y >= absNormal.x && absNormal.y >= absNormal.z) {
            return new Vec2[] {xz(a), xz(b), xz(c)};
        }
        if (absNormal.z >= absNormal.x && absNormal.z >= absNormal.y) {
            return new Vec2[] {xy(a), xy(b), xy(c)};
        }
        return new Vec2[] {yz(a), yz(b), yz(c)};
    }

    private static Vec2[] faceLocalUvs(Vec3 a, Vec3 b, Vec3 c) {
        Vec3 ab = b.subtract(a);
        Vec3 ac = c.subtract(a);
        double abLength = ab.length();
        if (abLength < 1.0e-12) {
            return new Vec2[] {new Vec2(0.0, 0.0), new Vec2(0.0, 0.0), new Vec2(0.0, 0.0)};
        }

        Vec3 uAxis = ab.divide(abLength);
        double cU = ac.dot(uAxis);
        double cV2 = Math.max(ac.lengthSquared() - cU * cU, 0.0);
        double cV = Math.sqrt(cV2);

        double scale = 1.0 / UV_DOMAIN_SIZE;
        return new Vec2[] {
                new Vec2(0.0, 0.0),
                new Vec2(abLength * scale, 0.0),
                new Vec2(cU * scale, cV * scale)
        };
    }

    private static Vec2 xy(Vec3 p) {
        return new Vec2(toUnitUv(p.x), toUnitUv(p.y));
    }

    private static Vec2 xz(Vec3 p) {
        return new Vec2(toUnitUv(p.x), toUnitUv(p.z));
    }

    private static Vec2 yz(Vec3 p) {
        return new Vec2(toUnitUv(p.y), toUnitUv(p.z));
    }

    private static double toUnitUv(double value) {
        return (value - UV_DOMAIN_MIN) / UV_DOMAIN_SIZE;
    }

    public static final class Result {
        private final List<Vec2> texCoords;
        private final int[] texCoordIndices;
        private final List<Vec3> triplanarPositions;
        private final List<Vec3> triplanarNormals;

        private Result(
                List<Vec2> texCoords,
                int[] texCoordIndices,
                List<Vec3> triplanarPositions,
                List<Vec3> triplanarNormals) {
            this.texCoords = Collections.unmodifiableList(texCoords);
            this.texCoordIndices = texCoordIndices;
            this.triplanarPositions = Collections.unmodifiableList(triplanarPositions);
            this.triplanarNormals = Collections.unmodifiableList(triplanarNormals);
        }

        public List<Vec2> getTexCoords() {
            return texCoords;
        }

        public int getTexCoordIndex(int faceIndex, int corner) {
            return texCoordIndices[faceIndex * 3 + corner];
        }

        public List<Vec3> getTriplanarPositions() {
            return triplanarPositions;
        }

        public List<Vec3> getTriplanarNormals() {
            return triplanarNormals;
        }
    }
}
