package com.example;

import javafx.application.Application;
import javafx.beans.binding.Bindings;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.PointLight;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.Slider;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.image.Image;
import javafx.scene.image.PixelReader;
import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.DrawMode;
import javafx.scene.shape.MeshView;
import javafx.stage.FileChooser;
import javafx.stage.Stage;

import java.io.File;

public class MainApp extends Application {
    private static final double GRID_DOMAIN_MIN = -1.5;
    private static final double GRID_DOMAIN_MAX = 1.5;
    private static final Color VIEW_BACKGROUND = Color.rgb(31, 34, 39);
    private static final Color AMBIENT_LIGHT_COLOR = Color.color(0.60, 0.62, 0.66);
    private static final Color KEY_LIGHT_COLOR = Color.color(0.44, 0.44, 0.48);
    private static final Color FILL_LIGHT_COLOR = Color.color(0.20, 0.22, 0.26);
    private static final Color MESH_DIFFUSE_COLOR = Color.rgb(104, 148, 208);
    private static final Color MESH_SPECULAR_COLOR = Color.color(0.12, 0.13, 0.15);

    private final PhongMaterial meshMaterial = new PhongMaterial();
    private final MeshView meshView = new MeshView();
    private final ComboBox<String> sdfShapeBox = new ComboBox<>();
    private final ComboBox<NormalMode> normalModeBox = new ComboBox<>();
    private final ComboBox<UvGenerator.Mode> uvModeBox = new ComboBox<>();
    private final Slider gridResolutionSlider = new Slider(8.0, 96.0, 32.0);
    private final Slider rotationAngleSlider = new Slider(0.0, 90.0, 25.0);
    private final Slider autoSmoothAngleSlider = new Slider(5.0, 120.0, 45.0);
    private final Slider textureRepeatSlider = new Slider(1.0, 64.0, 8.0);
    private final Slider qefCenterBiasSlider = new Slider(0.0, 0.1, DualContouring.DEFAULT_QEF_CENTER_BIAS);
    private final Slider vertexSlackSlider = new Slider(0.0, 2.0, DualContouring.DEFAULT_VERTEX_SLACK);
    private final Slider qefErrorFallbackThresholdSlider =
            new Slider(0.0, 10.0, DualContouring.DEFAULT_QEF_ERROR_FALLBACK_THRESHOLD);
    private final Slider projectionIterationsSlider =
            new Slider(0.0, 8.0, DualContouring.DEFAULT_PROJECTION_ITERATIONS);
    private final Slider maxProjectionStepSlider =
            new Slider(0.05, 2.0, DualContouring.DEFAULT_MAX_PROJECTION_STEP);
    private final Slider projectionStrengthSlider =
            new Slider(0.0, 1.0, DualContouring.DEFAULT_PROJECTION_STRENGTH);
    private final CheckBox wireframeCheckBox = new CheckBox("Wireframe");
    private final CheckBox binarySearchCheckBox = new CheckBox("Binary zero crossing");
    private final CheckBox flipWindingCheckBox = new CheckBox("Flip winding");
    private final CheckBox hardClampToCellCheckBox = new CheckBox("Hard clamp to cell");
    private final CheckBox useMassPointFallbackCheckBox = new CheckBox("Mass point fallback");
    private final CheckBox enableSurfaceProjectionCheckBox = new CheckBox("Surface projection");
    private final CheckBox applyTextureCheckBox = new CheckBox("Apply texture");
    private final Label shapeDescriptionLabel = new Label();
    private final Label texturePathLabel = new Label();
    private final Label statusLabel = new Label();

    private Sdf activeSdf;
    private File textureFile = findDefaultTextureFile();
    private Image generatedTextureImage;
    private boolean usingGeneratedTexture;

    @Override
    public void start(Stage stage) {
        Group world = new Group();
        configureMeshView();

        AmbientLight ambientLight = new AmbientLight(AMBIENT_LIGHT_COLOR);

        PointLight keyLight = new PointLight(KEY_LIGHT_COLOR);
        keyLight.setTranslateX(-3.0);
        keyLight.setTranslateY(-4.0);
        keyLight.setTranslateZ(-5.0);

        PointLight fillLight = new PointLight(FILL_LIGHT_COLOR);
        fillLight.setTranslateX(4.0);
        fillLight.setTranslateY(2.0);
        fillLight.setTranslateZ(5.0);

        world.getChildren().addAll(meshView, ambientLight, keyLight, fillLight);

        PerspectiveCamera camera = new PerspectiveCamera(true);
        CameraController cameraController = new CameraController(camera);

        Group subSceneRoot = new Group(world, cameraController.getRig());
        SubScene subScene = new SubScene(subSceneRoot, 900.0, 700.0, true, SceneAntialiasing.BALANCED);
        subScene.setFill(VIEW_BACKGROUND);
        subScene.setCamera(camera);
        cameraController.attach(subScene);

        StackPane viewport = new StackPane(subScene);
        viewport.setStyle("-fx-background-color: #1f2227;");
        subScene.widthProperty().bind(viewport.widthProperty());
        subScene.heightProperty().bind(viewport.heightProperty());

        BorderPane root = new BorderPane();
        root.setCenter(viewport);
        root.setRight(createControls(stage));

        Scene scene = new Scene(root, 1120.0, 760.0, true);
        stage.setTitle("SDF Dual Contouring CPU Demo");
        stage.setScene(scene);
        stage.show();

        rebuildMesh();
    }

    private ScrollPane createControls(Stage stage) {
        sdfShapeBox.getItems().addAll(
                "Sphere",
                "RotatedCube",
                "Pyramid",
                "Torus",
                "ThinRotatedBox",
                "BoxMinusSphere",
                "DoubleTorusOrUnion",
                "PyramidOnCube");
        sdfShapeBox.getSelectionModel().select("Sphere");
        shapeDescriptionLabel.setWrapText(true);
        shapeDescriptionLabel.setMinHeight(80.0);
        shapeDescriptionLabel.setStyle("-fx-text-fill: #3b4652;");
        updateShapeDescription();
        sdfShapeBox.valueProperty().addListener((observable, oldValue, newValue) -> updateShapeDescription());

        normalModeBox.getItems().addAll(NormalMode.FLAT, NormalMode.SDF_GRADIENT, NormalMode.AUTO_SMOOTH);
        normalModeBox.getSelectionModel().select(NormalMode.AUTO_SMOOTH);
        uvModeBox.getItems().addAll(
                UvGenerator.Mode.PLANAR_XY,
                UvGenerator.Mode.PLANAR_XZ,
                UvGenerator.Mode.PLANAR_YZ,
                UvGenerator.Mode.BOX_PROJECTED,
                UvGenerator.Mode.FACE_LOCAL,
                UvGenerator.Mode.TRIPLANAR_READY);
        uvModeBox.getSelectionModel().select(UvGenerator.Mode.BOX_PROJECTED);

        configureIntegerSlider(gridResolutionSlider, 8.0);
        configureAngleSlider(rotationAngleSlider);
        configureDecimalSlider(autoSmoothAngleSlider, 15.0, 2, 1.0);
        configureIntegerSlider(textureRepeatSlider, 8.0);
        configureDecimalSlider(qefCenterBiasSlider, 0.025, 4, 0.001);
        configureDecimalSlider(vertexSlackSlider, 0.5, 4, 0.1);
        configureDecimalSlider(qefErrorFallbackThresholdSlider, 2.0, 1, 0.1);
        configureIntegerSlider(projectionIterationsSlider, 1.0);
        configureDecimalSlider(maxProjectionStepSlider, 0.5, 4, 0.05);
        configureDecimalSlider(projectionStrengthSlider, 0.25, 4, 0.05);

        Label resolutionValue = new Label();
        resolutionValue.textProperty().bind(Bindings.createStringBinding(
                () -> Integer.toString((int) Math.round(gridResolutionSlider.getValue())),
                gridResolutionSlider.valueProperty()));

        Label rotationValue = new Label();
        rotationValue.textProperty().bind(Bindings.createStringBinding(
                () -> String.format("%.0f deg", rotationAngleSlider.getValue()),
                rotationAngleSlider.valueProperty()));

        Label autoSmoothAngleValue = new Label();
        autoSmoothAngleValue.textProperty().bind(Bindings.createStringBinding(
                () -> String.format("%.0f deg", autoSmoothAngleSlider.getValue()),
                autoSmoothAngleSlider.valueProperty()));

        Label normalWarningLabel = new Label(
                "Smooth normals can hide geometry errors; use FLAT mode when debugging sharp-feature preservation.");
        normalWarningLabel.setWrapText(true);
        normalWarningLabel.setStyle("-fx-text-fill: #7a4f00;");

        Label textureRepeatValue = new Label();
        textureRepeatValue.textProperty().bind(Bindings.createStringBinding(
                () -> Integer.toString(getTextureRepeatCount()),
                textureRepeatSlider.valueProperty()));

        Label qefCenterBiasValue = new Label();
        qefCenterBiasValue.textProperty().bind(Bindings.createStringBinding(
                () -> String.format("%.4f", qefCenterBiasSlider.getValue()),
                qefCenterBiasSlider.valueProperty()));

        Label vertexSlackValue = new Label();
        vertexSlackValue.textProperty().bind(Bindings.createStringBinding(
                () -> String.format("%.2f cells", vertexSlackSlider.getValue()),
                vertexSlackSlider.valueProperty()));

        Label qefErrorThresholdValue = new Label();
        qefErrorThresholdValue.textProperty().bind(Bindings.createStringBinding(
                () -> String.format("%.2f", qefErrorFallbackThresholdSlider.getValue()),
                qefErrorFallbackThresholdSlider.valueProperty()));

        Label projectionIterationsValue = new Label();
        projectionIterationsValue.textProperty().bind(Bindings.createStringBinding(
                () -> Integer.toString((int) Math.round(projectionIterationsSlider.getValue())),
                projectionIterationsSlider.valueProperty()));

        Label maxProjectionStepValue = new Label();
        maxProjectionStepValue.textProperty().bind(Bindings.createStringBinding(
                () -> String.format("%.2f cells", maxProjectionStepSlider.getValue()),
                maxProjectionStepSlider.valueProperty()));

        Label projectionStrengthValue = new Label();
        projectionStrengthValue.textProperty().bind(Bindings.createStringBinding(
                () -> String.format("%.2f", projectionStrengthSlider.getValue()),
                projectionStrengthSlider.valueProperty()));

        Button rebuildButton = new Button("Rebuild Mesh");
        rebuildButton.setMaxWidth(Double.MAX_VALUE);
        rebuildButton.setOnAction(event -> rebuildMesh());

        Button chooseTextureButton = new Button("Choose Texture");
        chooseTextureButton.setMaxWidth(Double.MAX_VALUE);
        chooseTextureButton.setOnAction(event -> chooseTexture(stage));

        Button checkerboardButton = new Button("Use Checkerboard");
        checkerboardButton.setMaxWidth(Double.MAX_VALUE);
        checkerboardButton.setOnAction(event -> useGeneratedCheckerboardTexture());

        binarySearchCheckBox.setSelected(true);
        useMassPointFallbackCheckBox.setSelected(true);
        wireframeCheckBox.selectedProperty().addListener((observable, oldValue, selected) -> updateDrawMode());
        applyTextureCheckBox.selectedProperty().addListener((observable, oldValue, selected) -> updateMaterial());
        textureRepeatSlider.valueProperty().addListener((observable, oldValue, selected) -> updateMaterial());
        texturePathLabel.setWrapText(true);
        texturePathLabel.setStyle("-fx-text-fill: #3b4652;");
        updateTexturePathLabel();

        VBox controls = new VBox(10.0,
                new Label("SDF Shape"),
                sdfShapeBox,
                shapeDescriptionLabel,
                new Label("Grid Resolution"),
                resolutionValue,
                gridResolutionSlider,
                new Label("Rotation Angle"),
                rotationValue,
                rotationAngleSlider,
                new Label("Normal Mode"),
                normalModeBox,
                normalWarningLabel,
                new Label("Auto Smooth Angle"),
                autoSmoothAngleValue,
                autoSmoothAngleSlider,
                new Label("UV Mode"),
                uvModeBox,
                applyTextureCheckBox,
                chooseTextureButton,
                checkerboardButton,
                new Label("Texture Repeat"),
                textureRepeatValue,
                textureRepeatSlider,
                texturePathLabel,
                new Label("QEF Center Bias"),
                qefCenterBiasValue,
                qefCenterBiasSlider,
                new Label("Vertex Slack"),
                vertexSlackValue,
                vertexSlackSlider,
                new Label("QEF Error Fallback"),
                qefErrorThresholdValue,
                qefErrorFallbackThresholdSlider,
                wireframeCheckBox,
                binarySearchCheckBox,
                flipWindingCheckBox,
                hardClampToCellCheckBox,
                useMassPointFallbackCheckBox,
                enableSurfaceProjectionCheckBox,
                new Label("Projection Iterations"),
                projectionIterationsValue,
                projectionIterationsSlider,
                new Label("Max Projection Step"),
                maxProjectionStepValue,
                maxProjectionStepSlider,
                new Label("Projection Strength"),
                projectionStrengthValue,
                projectionStrengthSlider,
                rebuildButton,
                statusLabel);
        controls.setAlignment(Pos.TOP_LEFT);
        controls.setPadding(new Insets(16.0));
        controls.setPrefWidth(260.0);
        controls.setMinWidth(240.0);
        controls.setStyle("-fx-background-color: #f4f5f7; -fx-border-color: #d3d7df; -fx-border-width: 0 0 0 1;");
        VBox.setVgrow(statusLabel, Priority.ALWAYS);

        sdfShapeBox.setMaxWidth(Double.MAX_VALUE);
        normalModeBox.setMaxWidth(Double.MAX_VALUE);
        uvModeBox.setMaxWidth(Double.MAX_VALUE);
        gridResolutionSlider.setMaxWidth(Double.MAX_VALUE);
        rotationAngleSlider.setMaxWidth(Double.MAX_VALUE);
        autoSmoothAngleSlider.setMaxWidth(Double.MAX_VALUE);
        textureRepeatSlider.setMaxWidth(Double.MAX_VALUE);
        qefCenterBiasSlider.setMaxWidth(Double.MAX_VALUE);
        vertexSlackSlider.setMaxWidth(Double.MAX_VALUE);
        qefErrorFallbackThresholdSlider.setMaxWidth(Double.MAX_VALUE);
        projectionIterationsSlider.setMaxWidth(Double.MAX_VALUE);
        maxProjectionStepSlider.setMaxWidth(Double.MAX_VALUE);
        projectionStrengthSlider.setMaxWidth(Double.MAX_VALUE);

        ScrollPane scrollPane = new ScrollPane(controls);
        scrollPane.setFitToWidth(true);
        scrollPane.setPrefWidth(280.0);
        scrollPane.setMinWidth(260.0);
        scrollPane.setStyle("-fx-background-color: #f4f5f7; -fx-background: #f4f5f7;");
        return scrollPane;
    }

    private void configureMeshView() {
        meshMaterial.setSpecularColor(MESH_SPECULAR_COLOR);
        meshMaterial.setSpecularPower(18.0);
        meshView.setMaterial(meshMaterial);
        updateMaterial();
        meshView.setCullFace(CullFace.NONE);
    }

    private void configureIntegerSlider(Slider slider, double majorTickUnit) {
        slider.setShowTickLabels(true);
        slider.setShowTickMarks(true);
        slider.setMajorTickUnit(majorTickUnit);
        slider.setMinorTickCount((int) majorTickUnit - 1);
        slider.setBlockIncrement(1.0);
        slider.setSnapToTicks(true);
    }

    private void configureAngleSlider(Slider slider) {
        slider.setShowTickLabels(true);
        slider.setShowTickMarks(true);
        slider.setMajorTickUnit(15.0);
        slider.setMinorTickCount(2);
        slider.setBlockIncrement(1.0);
    }

    private void configureDecimalSlider(Slider slider, double majorTickUnit, int minorTickCount, double blockIncrement) {
        slider.setShowTickLabels(true);
        slider.setShowTickMarks(true);
        slider.setMajorTickUnit(majorTickUnit);
        slider.setMinorTickCount(minorTickCount);
        slider.setBlockIncrement(blockIncrement);
    }

    private void rebuildMesh() {
        activeSdf = createSelectedSdf();
        int resolution = (int) Math.round(gridResolutionSlider.getValue());
        MeshData meshData = DualContouring.generate(
                activeSdf,
                resolution,
                binarySearchCheckBox.isSelected(),
                flipWindingCheckBox.isSelected(),
                qefCenterBiasSlider.getValue(),
                vertexSlackSlider.getValue(),
                hardClampToCellCheckBox.isSelected(),
                useMassPointFallbackCheckBox.isSelected(),
                qefErrorFallbackThresholdSlider.getValue(),
                enableSurfaceProjectionCheckBox.isSelected(),
                (int) Math.round(projectionIterationsSlider.getValue()),
                maxProjectionStepSlider.getValue(),
                projectionStrengthSlider.getValue());

        meshView.setMesh(MeshBuilder.build(
                meshData,
                activeSdf,
                normalModeBox.getSelectionModel().getSelectedItem(),
                autoSmoothAngleSlider.getValue(),
                uvModeBox.getSelectionModel().getSelectedItem()));
        updateDrawMode();

        double sampleAtOrigin = activeSdf.eval(Vec3.zero());
        statusLabel.setText(String.format(
                "SDF(0,0,0): %.4f%nGrid: %d%nDomain: [%.1f, %.1f]^3%nActive cells: %d%nVertices: %d%nTriangles: %d%nQEF solved: %d%nMass fallback: %d%nHard clamps: %d%nSlack clamps: %d%nAvg QEF error: %.6f%nProjected: %d%nAvg |SDF| before: %.6f%nAvg |SDF| after: %.6f%nBuild: %d ms",
                sampleAtOrigin,
                resolution,
                GRID_DOMAIN_MIN,
                GRID_DOMAIN_MAX,
                meshData.getActiveCellCount(),
                meshData.getVertices().size(),
                meshData.getTriangles().size(),
                meshData.getQefSolvedCount(),
                meshData.getMassFallbackCount(),
                meshData.getHardClampCount(),
                meshData.getSlackClampCount(),
                meshData.getAverageQefError(),
                meshData.getProjectedVertexCount(),
                meshData.getAverageAbsSdfBeforeProjection(),
                meshData.getAverageAbsSdfAfterProjection(),
                meshData.getBuildTimeMillis()));
    }

    private Sdf createSelectedSdf() {
        String shape = sdfShapeBox.getSelectionModel().getSelectedItem();
        double angleRadians = Math.toRadians(rotationAngleSlider.getValue());
        if ("RotatedCube".equals(shape)) {
            return SdfLibrary.rotatedCube(angleRadians);
        }
        if ("Pyramid".equals(shape)) {
            return SdfLibrary.pyramid(1.3, 0.75);
        }
        if ("Torus".equals(shape)) {
            return SdfLibrary.torus(0.55, 0.22);
        }
        if ("ThinRotatedBox".equals(shape)) {
            return SdfLibrary.thinRotatedBox(angleRadians);
        }
        if ("BoxMinusSphere".equals(shape)) {
            return SdfLibrary.boxMinusSphere();
        }
        if ("DoubleTorusOrUnion".equals(shape)) {
            return SdfLibrary.doubleTorusOrUnion();
        }
        if ("PyramidOnCube".equals(shape)) {
            return SdfLibrary.pyramidOnCube();
        }
        return SdfLibrary.sphere(0.75);
    }

    private void updateShapeDescription() {
        shapeDescriptionLabel.setText(describeShape(sdfShapeBox.getSelectionModel().getSelectedItem()));
    }

    private String describeShape(String shape) {
        if ("RotatedCube".equals(shape)) {
            return "Rotated cube, half extents (0.65, 0.65, 0.65), rotated around Y and X by the UI angle. Tests grid misalignment and sharp edges.";
        }
        if ("Pyramid".equals(shape)) {
            return "Square-base pyramid centered at origin. Tests the apex and slanted sharp ridges.";
        }
        if ("Torus".equals(shape)) {
            return "Smooth torus, major 0.55 and minor 0.22. Tests Hermite normals, curved features, and surface projection.";
        }
        if ("ThinRotatedBox".equals(shape)) {
            return "Thin rotated box, half extents (0.85, 0.12, 0.55), with UI Y rotation plus 17 deg Z rotation. Tests thin features and grid-edge misses.";
        }
        if ("BoxMinusSphere".equals(shape)) {
            return "Cube minus sphere using max(box, -sphere). Tests CSG seams and non-smooth composed gradients.";
        }
        if ("DoubleTorusOrUnion".equals(shape)) {
            return "Union of two tori. Tests topology changes and multiple smooth curved components.";
        }
        if ("PyramidOnCube".equals(shape)) {
            return "Union of a cube base and pyramid top. Tests the sharp crease between primitives and non-smooth CSG gradients.";
        }
        return "Baseline sphere centered at origin. Useful for checking basic sampling, winding, and smooth normals.";
    }

    private void updateDrawMode() {
        meshView.setDrawMode(wireframeCheckBox.isSelected() ? DrawMode.LINE : DrawMode.FILL);
    }

    private void chooseTexture(Stage stage) {
        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Choose Texture");
        fileChooser.getExtensionFilters().addAll(
                new FileChooser.ExtensionFilter("Image Files", "*.png", "*.jpg", "*.jpeg", "*.gif"),
                new FileChooser.ExtensionFilter("All Files", "*.*"));
        if (textureFile != null && textureFile.getParentFile() != null && textureFile.getParentFile().isDirectory()) {
            fileChooser.setInitialDirectory(textureFile.getParentFile());
        }

        File selectedFile = fileChooser.showOpenDialog(stage);
        if (selectedFile != null) {
            textureFile = selectedFile;
            usingGeneratedTexture = false;
            applyTextureCheckBox.setSelected(true);
            updateTexturePathLabel();
            updateMaterial();
        }
    }

    private void useGeneratedCheckerboardTexture() {
        generatedTextureImage = null;
        usingGeneratedTexture = true;
        applyTextureCheckBox.setSelected(true);
        updateTexturePathLabel();
        updateMaterial();
    }

    private void updateMaterial() {
        meshMaterial.setSpecularColor(MESH_SPECULAR_COLOR);
        meshMaterial.setSpecularPower(18.0);

        if (applyTextureCheckBox.isSelected()) {
            Image texture = usingGeneratedTexture
                    ? getGeneratedCheckerboardTexture()
                    : loadTextureFile(textureFile);
            if (!texture.isError()) {
                meshMaterial.setDiffuseMap(texture);
                meshMaterial.setDiffuseColor(Color.WHITE);
                return;
            }
        }

        meshMaterial.setDiffuseMap(null);
        meshMaterial.setDiffuseColor(MESH_DIFFUSE_COLOR);
    }

    private void updateTexturePathLabel() {
        if (usingGeneratedTexture) {
            texturePathLabel.setText("Generated checkerboard");
        } else if (textureFile == null) {
            texturePathLabel.setText("No texture selected");
        } else {
            texturePathLabel.setText(textureFile.getPath());
        }
    }

    private Image loadTextureFile(File file) {
        if (file != null && file.isFile()) {
            Image source = new Image(file.toURI().toString());
            if (!source.isError()) {
                return repeatTexture(source, getTextureRepeatCount());
            }
        }
        return getGeneratedCheckerboardTexture();
    }

    private Image getGeneratedCheckerboardTexture() {
        generatedTextureImage = createCheckerboardTexture(512, getTextureRepeatCount());
        return generatedTextureImage;
    }

    private Image createCheckerboardTexture(int size, int repeatCount) {
        WritableImage image = new WritableImage(size, size);
        PixelWriter writer = image.getPixelWriter();
        int squareSize = Math.max(2, size / Math.max(2, repeatCount * 2));
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                boolean light = ((x / squareSize) + (y / squareSize)) % 2 == 0;
                writer.setColor(x, y, light ? Color.WHITE : Color.rgb(25, 25, 25));
            }
        }
        return image;
    }

    private Image repeatTexture(Image source, int repeatCount) {
        if (repeatCount <= 1) {
            return source;
        }

        PixelReader reader = source.getPixelReader();
        if (reader == null) {
            return source;
        }

        int outputSize = 1024;
        int sourceWidth = Math.max(1, (int) Math.round(source.getWidth()));
        int sourceHeight = Math.max(1, (int) Math.round(source.getHeight()));
        WritableImage repeated = new WritableImage(outputSize, outputSize);
        PixelWriter writer = repeated.getPixelWriter();

        /*
         * JavaFX PhongMaterial does not expose sampler wrap mode, so repeat
         * wrapping is approximated by baking a repeated texture into [0, 1] UV
         * space. This makes the texture appear smaller without changing DC
         * geometry or requiring shader support.
         */
        for (int y = 0; y < outputSize; y++) {
            int sourceY = (int) (((long) y * repeatCount * sourceHeight / outputSize) % sourceHeight);
            for (int x = 0; x < outputSize; x++) {
                int sourceX = (int) (((long) x * repeatCount * sourceWidth / outputSize) % sourceWidth);
                writer.setColor(x, y, reader.getColor(sourceX, sourceY));
            }
        }
        return repeated;
    }

    private int getTextureRepeatCount() {
        return Math.max(1, (int) Math.round(textureRepeatSlider.getValue()));
    }

    private static File findDefaultTextureFile() {
        File local = new File("metal.png");
        if (local.isFile()) {
            return local;
        }

        File nested = new File("smooth/metal.png");
        if (nested.isFile()) {
            return nested;
        }

        return local;
    }

    public static void main(String[] args) {
        launch(args);
    }
}
