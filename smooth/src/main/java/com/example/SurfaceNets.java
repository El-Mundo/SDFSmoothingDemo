package com.example;

import java.util.ArrayList;
import java.util.List;

public final class SurfaceNets {
    private static final double DOMAIN_MIN = DualContouring.DOMAIN_MIN;
    private static final double DOMAIN_MAX = DualContouring.DOMAIN_MAX;

    private static final int[][] CORNER_OFFSETS = {
            {0, 0, 0},
            {1, 0, 0},
            {0, 1, 0},
            {1, 1, 0},
            {0, 0, 1},
            {1, 0, 1},
            {0, 1, 1},
            {1, 1, 1}
    };

    private static final int[][] CELL_EDGES = {
            {0, 1}, {2, 3}, {4, 5}, {6, 7},
            {0, 2}, {1, 3}, {4, 6}, {5, 7},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    private SurfaceNets() {
    }

    public static MeshData generate(
            Sdf sdf,
            int resolution,
            boolean useBinarySearchZeroCrossing,
            boolean flipWinding,
            boolean enableSurfaceProjection,
            int projectionIterations,
            double maxProjectionStep,
            double projectionStrength) {
        if (resolution < 1) {
            throw new IllegalArgumentException("Resolution must be at least 1.");
        }

        long startTime = System.nanoTime();
        double cellSize = (DOMAIN_MAX - DOMAIN_MIN) / resolution;
        double normalEpsilon = Math.max(cellSize * 0.01, 1.0e-5);
        ProjectionOptions projectionOptions = new ProjectionOptions(
                enableSurfaceProjection,
                projectionIterations,
                maxProjectionStep,
                projectionStrength);
        ProjectionStats projectionStats = new ProjectionStats();

        double[] gridValues = sampleGrid(sdf, resolution, cellSize);
        CellData[] cells = new CellData[resolution * resolution * resolution];
        List<Vec3> vertices = new ArrayList<>();

        /*
         * Naive Surface Nets also places one vertex per active cell, but uses
         * the average of sign-changing grid-edge crossings instead of a QEF.
         * This keeps the topology comparable to the Dual Contouring path while
         * making feature-placement differences easy to inspect.
         */
        int activeCellCount = 0;
        for (int z = 0; z < resolution; z++) {
            for (int y = 0; y < resolution; y++) {
                for (int x = 0; x < resolution; x++) {
                    CellData cell = buildCell(
                            sdf,
                            resolution,
                            cellSize,
                            normalEpsilon,
                            gridValues,
                            x,
                            y,
                            z,
                            useBinarySearchZeroCrossing,
                            projectionOptions,
                            projectionStats);
                    if (cell != null && cell.getDualVertex() != null) {
                        cell.setVertexIndex(vertices.size());
                        vertices.add(cell.getDualVertex());
                        activeCellCount++;
                    }
                    cells[cellIndex(x, y, z, resolution)] = cell;
                }
            }
        }

        List<int[]> triangles = new ArrayList<>();
        emitQuads(
                sdf,
                resolution,
                cellSize,
                normalEpsilon,
                gridValues,
                cells,
                vertices,
                triangles,
                useBinarySearchZeroCrossing,
                flipWinding);

        long buildTimeMillis = (System.nanoTime() - startTime) / 1_000_000L;
        return new MeshData(
                vertices,
                triangles,
                activeCellCount,
                buildTimeMillis,
                0,
                0,
                0,
                0,
                0.0,
                projectionStats.projectedVertexCount,
                projectionStats.averageAbsSdfBefore(),
                projectionStats.averageAbsSdfAfter());
    }

    private static double[] sampleGrid(Sdf sdf, int resolution, double cellSize) {
        int gridSize = resolution + 1;
        double[] values = new double[gridSize * gridSize * gridSize];
        for (int z = 0; z < gridSize; z++) {
            for (int y = 0; y < gridSize; y++) {
                for (int x = 0; x < gridSize; x++) {
                    values[gridIndex(x, y, z, resolution)] = sdf.eval(gridPosition(x, y, z, cellSize));
                }
            }
        }
        return values;
    }

    private static CellData buildCell(
            Sdf sdf,
            int resolution,
            double cellSize,
            double normalEpsilon,
            double[] gridValues,
            int cellX,
            int cellY,
            int cellZ,
            boolean useBinarySearchZeroCrossing,
            ProjectionOptions projectionOptions,
            ProjectionStats projectionStats) {
        double[] cornerValues = new double[8];
        Vec3[] cornerPositions = new Vec3[8];
        boolean hasInside = false;
        boolean hasOutside = false;

        for (int corner = 0; corner < 8; corner++) {
            int[] offset = CORNER_OFFSETS[corner];
            int x = cellX + offset[0];
            int y = cellY + offset[1];
            int z = cellZ + offset[2];
            double value = gridValues[gridIndex(x, y, z, resolution)];
            cornerValues[corner] = value;
            cornerPositions[corner] = gridPosition(x, y, z, cellSize);

            if (isInside(value)) {
                hasInside = true;
            } else {
                hasOutside = true;
            }
        }

        if (!hasInside || !hasOutside) {
            return null;
        }

        Vec3 crossingSum = Vec3.zero();
        int crossingCount = 0;
        for (int[] edge : CELL_EDGES) {
            int a = edge[0];
            int b = edge[1];
            if (!hasSignChange(cornerValues[a], cornerValues[b])) {
                continue;
            }

            Vec3 crossing = findZeroCrossing(
                    sdf,
                    cornerPositions[a],
                    cornerValues[a],
                    cornerPositions[b],
                    cornerValues[b],
                    useBinarySearchZeroCrossing);
            crossingSum = crossingSum.add(crossing);
            crossingCount++;
        }

        if (crossingCount == 0) {
            return null;
        }

        CellData cell = new CellData(cellX, cellY, cellZ);
        Vec3 vertex = crossingSum.divide(crossingCount);
        vertex = projectToSurface(
                sdf,
                vertex,
                cellSize,
                normalEpsilon,
                projectionOptions,
                projectionStats);
        cell.setDualVertex(vertex);
        return cell;
    }

    private static Vec3 projectToSurface(
            Sdf sdf,
            Vec3 initial,
            double cellSize,
            double normalEpsilon,
            ProjectionOptions options,
            ProjectionStats stats) {
        if (!options.enabled || options.iterations <= 0 || options.strength <= 0.0) {
            return initial;
        }

        Vec3 p = initial;
        double before = Math.abs(sdf.eval(p));
        boolean projected = false;
        double maxStepLength = options.maxStep * cellSize;

        for (int i = 0; i < options.iterations; i++) {
            double d = sdf.eval(p);
            Vec3 gradient = centralDifferenceGradient(sdf, p, normalEpsilon);
            double gradientLength = gradient.length();
            if (gradientLength < 1.0e-12) {
                break;
            }

            Vec3 normal = gradient.divide(gradientLength);
            Vec3 step = normal.multiply(d * options.strength);
            double stepLength = step.length();
            if (stepLength > maxStepLength) {
                step = step.divide(stepLength).multiply(maxStepLength);
            }

            if (step.lengthSquared() < 1.0e-24) {
                break;
            }

            p = p.subtract(step);
            projected = true;
        }

        if (projected) {
            stats.add(before, Math.abs(sdf.eval(p)));
        }

        return p;
    }

    private static void emitQuads(
            Sdf sdf,
            int resolution,
            double cellSize,
            double normalEpsilon,
            double[] gridValues,
            CellData[] cells,
            List<Vec3> vertices,
            List<int[]> triangles,
            boolean useBinarySearchZeroCrossing,
            boolean flipWinding) {
        for (int z = 1; z < resolution; z++) {
            for (int y = 1; y < resolution; y++) {
                for (int x = 0; x < resolution; x++) {
                    emitXEdgeQuad(sdf, resolution, cellSize, normalEpsilon, gridValues, cells,
                            vertices, triangles, x, y, z, useBinarySearchZeroCrossing, flipWinding);
                }
            }
        }

        for (int z = 1; z < resolution; z++) {
            for (int y = 0; y < resolution; y++) {
                for (int x = 1; x < resolution; x++) {
                    emitYEdgeQuad(sdf, resolution, cellSize, normalEpsilon, gridValues, cells,
                            vertices, triangles, x, y, z, useBinarySearchZeroCrossing, flipWinding);
                }
            }
        }

        for (int z = 0; z < resolution; z++) {
            for (int y = 1; y < resolution; y++) {
                for (int x = 1; x < resolution; x++) {
                    emitZEdgeQuad(sdf, resolution, cellSize, normalEpsilon, gridValues, cells,
                            vertices, triangles, x, y, z, useBinarySearchZeroCrossing, flipWinding);
                }
            }
        }
    }

    private static void emitXEdgeQuad(
            Sdf sdf,
            int resolution,
            double cellSize,
            double normalEpsilon,
            double[] gridValues,
            CellData[] cells,
            List<Vec3> vertices,
            List<int[]> triangles,
            int x,
            int y,
            int z,
            boolean useBinarySearchZeroCrossing,
            boolean flipWinding) {
        double a = gridValues[gridIndex(x, y, z, resolution)];
        double b = gridValues[gridIndex(x + 1, y, z, resolution)];
        if (!hasSignChange(a, b)) {
            return;
        }

        emitQuadIfReady(
                sdf,
                normalEpsilon,
                vertices,
                triangles,
                useBinarySearchZeroCrossing,
                flipWinding,
                gridPosition(x, y, z, cellSize),
                a,
                gridPosition(x + 1, y, z, cellSize),
                b,
                cell(cells, x, y - 1, z - 1, resolution),
                cell(cells, x, y, z - 1, resolution),
                cell(cells, x, y, z, resolution),
                cell(cells, x, y - 1, z, resolution));
    }

    private static void emitYEdgeQuad(
            Sdf sdf,
            int resolution,
            double cellSize,
            double normalEpsilon,
            double[] gridValues,
            CellData[] cells,
            List<Vec3> vertices,
            List<int[]> triangles,
            int x,
            int y,
            int z,
            boolean useBinarySearchZeroCrossing,
            boolean flipWinding) {
        double a = gridValues[gridIndex(x, y, z, resolution)];
        double b = gridValues[gridIndex(x, y + 1, z, resolution)];
        if (!hasSignChange(a, b)) {
            return;
        }

        emitQuadIfReady(
                sdf,
                normalEpsilon,
                vertices,
                triangles,
                useBinarySearchZeroCrossing,
                flipWinding,
                gridPosition(x, y, z, cellSize),
                a,
                gridPosition(x, y + 1, z, cellSize),
                b,
                cell(cells, x - 1, y, z - 1, resolution),
                cell(cells, x, y, z - 1, resolution),
                cell(cells, x, y, z, resolution),
                cell(cells, x - 1, y, z, resolution));
    }

    private static void emitZEdgeQuad(
            Sdf sdf,
            int resolution,
            double cellSize,
            double normalEpsilon,
            double[] gridValues,
            CellData[] cells,
            List<Vec3> vertices,
            List<int[]> triangles,
            int x,
            int y,
            int z,
            boolean useBinarySearchZeroCrossing,
            boolean flipWinding) {
        double a = gridValues[gridIndex(x, y, z, resolution)];
        double b = gridValues[gridIndex(x, y, z + 1, resolution)];
        if (!hasSignChange(a, b)) {
            return;
        }

        emitQuadIfReady(
                sdf,
                normalEpsilon,
                vertices,
                triangles,
                useBinarySearchZeroCrossing,
                flipWinding,
                gridPosition(x, y, z, cellSize),
                a,
                gridPosition(x, y, z + 1, cellSize),
                b,
                cell(cells, x - 1, y - 1, z, resolution),
                cell(cells, x, y - 1, z, resolution),
                cell(cells, x, y, z, resolution),
                cell(cells, x - 1, y, z, resolution));
    }

    private static void emitQuadIfReady(
            Sdf sdf,
            double normalEpsilon,
            List<Vec3> vertices,
            List<int[]> triangles,
            boolean useBinarySearchZeroCrossing,
            boolean flipWinding,
            Vec3 edgeA,
            double valueA,
            Vec3 edgeB,
            double valueB,
            CellData c0,
            CellData c1,
            CellData c2,
            CellData c3) {
        if (!isActive(c0) || !isActive(c1) || !isActive(c2) || !isActive(c3)) {
            return;
        }

        int i0 = c0.getVertexIndex();
        int i1 = c1.getVertexIndex();
        int i2 = c2.getVertexIndex();
        int i3 = c3.getVertexIndex();

        Vec3 crossing = findZeroCrossing(sdf, edgeA, valueA, edgeB, valueB, useBinarySearchZeroCrossing);
        Vec3 targetNormal = estimateNormal(sdf, crossing, normalEpsilon);

        double diagonal02 = vertices.get(i0).subtract(vertices.get(i2)).lengthSquared();
        double diagonal13 = vertices.get(i1).subtract(vertices.get(i3)).lengthSquared();
        if (diagonal02 <= diagonal13) {
            addTriangle(vertices, triangles, i0, i1, i2, targetNormal, flipWinding);
            addTriangle(vertices, triangles, i0, i2, i3, targetNormal, flipWinding);
        } else {
            addTriangle(vertices, triangles, i0, i1, i3, targetNormal, flipWinding);
            addTriangle(vertices, triangles, i1, i2, i3, targetNormal, flipWinding);
        }
    }

    private static void addTriangle(
            List<Vec3> vertices,
            List<int[]> triangles,
            int i0,
            int i1,
            int i2,
            Vec3 targetNormal,
            boolean flipWinding) {
        Vec3 a = vertices.get(i0);
        Vec3 b = vertices.get(i1);
        Vec3 c = vertices.get(i2);
        Vec3 geometricNormal = b.subtract(a).cross(c.subtract(a));

        if (geometricNormal.dot(targetNormal) < 0.0) {
            int tmp = i1;
            i1 = i2;
            i2 = tmp;
        }

        if (flipWinding) {
            int tmp = i1;
            i1 = i2;
            i2 = tmp;
        }

        triangles.add(new int[] {i0, i1, i2});
    }

    private static Vec3 findZeroCrossing(
            Sdf sdf,
            Vec3 a,
            double valueA,
            Vec3 b,
            double valueB,
            boolean useBinarySearchZeroCrossing) {
        if (Math.abs(valueA) < 1.0e-12) {
            return a;
        }
        if (Math.abs(valueB) < 1.0e-12) {
            return b;
        }
        if (!useBinarySearchZeroCrossing) {
            return interpolate(a, valueA, b, valueB);
        }

        Vec3 lo = a;
        Vec3 hi = b;
        double loValue = valueA;
        double hiValue = valueB;
        boolean loInside = isInside(loValue);

        for (int i = 0; i < 14; i++) {
            Vec3 mid = lo.add(hi).multiply(0.5);
            double midValue = sdf.eval(mid);
            if (isInside(midValue) == loInside) {
                lo = mid;
                loValue = midValue;
            } else {
                hi = mid;
                hiValue = midValue;
            }
        }

        return interpolate(lo, loValue, hi, hiValue);
    }

    private static Vec3 interpolate(Vec3 a, double valueA, Vec3 b, double valueB) {
        double denominator = valueA - valueB;
        if (Math.abs(denominator) < 1.0e-12) {
            return a.add(b).multiply(0.5);
        }
        double t = clamp(valueA / denominator, 0.0, 1.0);
        return a.add(b.subtract(a).multiply(t));
    }

    private static Vec3 estimateNormal(Sdf sdf, Vec3 p, double eps) {
        return centralDifferenceGradient(sdf, p, eps).normalize();
    }

    private static Vec3 centralDifferenceGradient(Sdf sdf, Vec3 p, double eps) {
        double nx = sdf.eval(new Vec3(p.x + eps, p.y, p.z)) - sdf.eval(new Vec3(p.x - eps, p.y, p.z));
        double ny = sdf.eval(new Vec3(p.x, p.y + eps, p.z)) - sdf.eval(new Vec3(p.x, p.y - eps, p.z));
        double nz = sdf.eval(new Vec3(p.x, p.y, p.z + eps)) - sdf.eval(new Vec3(p.x, p.y, p.z - eps));
        return new Vec3(nx, ny, nz);
    }

    private static boolean isInside(double value) {
        return value < 0.0;
    }

    private static boolean hasSignChange(double a, double b) {
        return isInside(a) != isInside(b);
    }

    private static boolean isActive(CellData cell) {
        return cell != null && cell.isActive();
    }

    private static CellData cell(CellData[] cells, int x, int y, int z, int resolution) {
        return cells[cellIndex(x, y, z, resolution)];
    }

    private static int cellIndex(int x, int y, int z, int resolution) {
        return (z * resolution + y) * resolution + x;
    }

    private static int gridIndex(int x, int y, int z, int resolution) {
        int gridSize = resolution + 1;
        return (z * gridSize + y) * gridSize + x;
    }

    private static Vec3 gridPosition(int x, int y, int z, double cellSize) {
        return new Vec3(
                DOMAIN_MIN + x * cellSize,
                DOMAIN_MIN + y * cellSize,
                DOMAIN_MIN + z * cellSize);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static final class ProjectionOptions {
        private final boolean enabled;
        private final int iterations;
        private final double maxStep;
        private final double strength;

        private ProjectionOptions(boolean enabled, int iterations, double maxStep, double strength) {
            this.enabled = enabled;
            this.iterations = Math.max(iterations, 0);
            this.maxStep = Math.max(maxStep, 0.0);
            this.strength = clamp(strength, 0.0, 1.0);
        }
    }

    private static final class ProjectionStats {
        private int projectedVertexCount;
        private double totalAbsSdfBefore;
        private double totalAbsSdfAfter;

        private void add(double absSdfBefore, double absSdfAfter) {
            projectedVertexCount++;
            totalAbsSdfBefore += absSdfBefore;
            totalAbsSdfAfter += absSdfAfter;
        }

        private double averageAbsSdfBefore() {
            if (projectedVertexCount == 0) {
                return 0.0;
            }
            return totalAbsSdfBefore / projectedVertexCount;
        }

        private double averageAbsSdfAfter() {
            if (projectedVertexCount == 0) {
                return 0.0;
            }
            return totalAbsSdfAfter / projectedVertexCount;
        }
    }
}
