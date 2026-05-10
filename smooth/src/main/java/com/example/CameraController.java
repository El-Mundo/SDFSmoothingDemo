package com.example;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;

public final class CameraController {
    private final PerspectiveCamera camera;
    private final Group rig = new Group();
    private final Translate pan = new Translate();
    private final Rotate yaw = new Rotate(-35.0, Rotate.Y_AXIS);
    private final Rotate pitch = new Rotate(-25.0, Rotate.X_AXIS);

    private double distance = 6.0;
    private double lastMouseX;
    private double lastMouseY;

    public CameraController(PerspectiveCamera camera) {
        this.camera = camera;
        camera.setNearClip(0.01);
        camera.setFarClip(200.0);
        camera.setFieldOfView(35.0);
        camera.setTranslateZ(-distance);

        rig.getTransforms().addAll(pan, yaw, pitch);
        rig.getChildren().add(camera);
    }

    public Group getRig() {
        return rig;
    }

    public PerspectiveCamera getCamera() {
        return camera;
    }

    public void attach(Node target) {
        target.addEventHandler(MouseEvent.MOUSE_PRESSED, this::handleMousePressed);
        target.addEventHandler(MouseEvent.MOUSE_DRAGGED, this::handleMouseDragged);
        target.addEventHandler(ScrollEvent.SCROLL, this::handleScroll);
    }

    private void handleMousePressed(MouseEvent event) {
        lastMouseX = event.getSceneX();
        lastMouseY = event.getSceneY();
        event.consume();
    }

    private void handleMouseDragged(MouseEvent event) {
        double dx = event.getSceneX() - lastMouseX;
        double dy = event.getSceneY() - lastMouseY;
        lastMouseX = event.getSceneX();
        lastMouseY = event.getSceneY();

        if (event.isPrimaryButtonDown()) {
            yaw.setAngle(yaw.getAngle() + dx * 0.35);
            pitch.setAngle(clamp(pitch.getAngle() - dy * 0.35, -85.0, 85.0));
            event.consume();
        } else if (event.isSecondaryButtonDown()) {
            pan(dx, dy);
            event.consume();
        }
    }

    private void handleScroll(ScrollEvent event) {
        double zoomFactor = Math.pow(0.92, event.getDeltaY() / 40.0);
        distance = clamp(distance * zoomFactor, 1.2, 40.0);
        camera.setTranslateZ(-distance);
        event.consume();
    }

    private void pan(double dx, double dy) {
        double scale = distance * 0.0018;
        double yawRadians = Math.toRadians(yaw.getAngle());
        double rightX = Math.cos(yawRadians);
        double rightZ = -Math.sin(yawRadians);

        pan.setX(pan.getX() + dx * scale * rightX);
        pan.setY(pan.getY() + dy * scale);
        pan.setZ(pan.getZ() + dx * scale * rightZ);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
