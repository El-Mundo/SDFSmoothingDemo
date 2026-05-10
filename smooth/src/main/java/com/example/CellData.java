package com.example;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class CellData {
    public final int x;
    public final int y;
    public final int z;

    private final List<HermiteSample> hermiteSamples = new ArrayList<>();
    private Vec3 dualVertex;
    private int vertexIndex = -1;

    public CellData(int x, int y, int z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void addHermiteSample(HermiteSample sample) {
        hermiteSamples.add(sample);
    }

    public List<HermiteSample> getHermiteSamples() {
        return Collections.unmodifiableList(hermiteSamples);
    }

    public boolean isActive() {
        return dualVertex != null && vertexIndex >= 0;
    }

    public Vec3 getDualVertex() {
        return dualVertex;
    }

    public void setDualVertex(Vec3 dualVertex) {
        this.dualVertex = dualVertex;
    }

    public int getVertexIndex() {
        return vertexIndex;
    }

    public void setVertexIndex(int vertexIndex) {
        this.vertexIndex = vertexIndex;
    }
}
