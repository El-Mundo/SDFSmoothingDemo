package com.example;

import java.util.List;

public final class QefSolver {
    private static final double PIVOT_EPSILON = 1.0e-10;

    private QefSolver() {
    }

    public static Result solve(List<HermiteSample> samples, Vec3 cellCenter, double centerBias) {
        if (samples.isEmpty()) {
            return Result.failed();
        }

        double[][] ata = new double[3][3];
        double[] atb = new double[3];

        for (HermiteSample sample : samples) {
            Vec3 n = sample.normal;
            double b = n.dot(sample.position);

            ata[0][0] += n.x * n.x;
            ata[0][1] += n.x * n.y;
            ata[0][2] += n.x * n.z;
            ata[1][0] += n.y * n.x;
            ata[1][1] += n.y * n.y;
            ata[1][2] += n.y * n.z;
            ata[2][0] += n.z * n.x;
            ata[2][1] += n.z * n.y;
            ata[2][2] += n.z * n.z;

            atb[0] += n.x * b;
            atb[1] += n.y * b;
            atb[2] += n.z * b;
        }

        double weight = Math.max(centerBias, 0.0);
        if (weight > 0.0) {
            ata[0][0] += weight;
            ata[1][1] += weight;
            ata[2][2] += weight;
            atb[0] += weight * cellCenter.x;
            atb[1] += weight * cellCenter.y;
            atb[2] += weight * cellCenter.z;
        }

        Vec3 qefPoint = solve3x3(ata, atb);
        if (qefPoint == null || !qefPoint.isFinite()) {
            return Result.failed();
        }
        return Result.solved(qefPoint, averageError(samples, qefPoint));
    }

    public static double averageError(List<HermiteSample> samples, Vec3 point) {
        if (samples.isEmpty()) {
            return 0.0;
        }

        double error = 0.0;
        for (HermiteSample sample : samples) {
            double planeDistance = sample.normal.dot(point.subtract(sample.position));
            error += planeDistance * planeDistance;
        }
        return error / samples.size();
    }

    public static Vec3 massPoint(List<HermiteSample> samples) {
        Vec3 sum = Vec3.zero();
        for (HermiteSample sample : samples) {
            sum = sum.add(sample.position);
        }
        return sum.divide(samples.size());
    }

    private static Vec3 solve3x3(double[][] matrix, double[] rhs) {
        double[][] augmented = {
                {matrix[0][0], matrix[0][1], matrix[0][2], rhs[0]},
                {matrix[1][0], matrix[1][1], matrix[1][2], rhs[1]},
                {matrix[2][0], matrix[2][1], matrix[2][2], rhs[2]}
        };

        for (int column = 0; column < 3; column++) {
            int pivotRow = column;
            double pivotSize = Math.abs(augmented[column][column]);
            for (int row = column + 1; row < 3; row++) {
                double candidateSize = Math.abs(augmented[row][column]);
                if (candidateSize > pivotSize) {
                    pivotRow = row;
                    pivotSize = candidateSize;
                }
            }

            if (pivotSize < PIVOT_EPSILON) {
                return null;
            }

            if (pivotRow != column) {
                double[] tmp = augmented[column];
                augmented[column] = augmented[pivotRow];
                augmented[pivotRow] = tmp;
            }

            for (int row = column + 1; row < 3; row++) {
                double factor = augmented[row][column] / augmented[column][column];
                for (int col = column; col < 4; col++) {
                    augmented[row][col] -= factor * augmented[column][col];
                }
            }
        }

        double[] x = new double[3];
        for (int row = 2; row >= 0; row--) {
            double value = augmented[row][3];
            for (int col = row + 1; col < 3; col++) {
                value -= augmented[row][col] * x[col];
            }
            x[row] = value / augmented[row][row];
        }

        return new Vec3(x[0], x[1], x[2]);
    }

    public static final class Result {
        private final Vec3 position;
        private final boolean solved;
        private final double averageError;

        private Result(Vec3 position, boolean solved, double averageError) {
            this.position = position;
            this.solved = solved;
            this.averageError = averageError;
        }

        public static Result solved(Vec3 position, double averageError) {
            return new Result(position, true, averageError);
        }

        public static Result failed() {
            return new Result(null, false, Double.NaN);
        }

        public Vec3 getPosition() {
            return position;
        }

        public boolean isSolved() {
            return solved;
        }

        public double getAverageError() {
            return averageError;
        }
    }
}
