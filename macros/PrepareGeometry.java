import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Properties;
import java.util.Set;
import java.util.regex.Pattern;

import star.base.neo.DoubleVector;
import star.common.GeometryPart;
import star.common.ImportManager;
import star.common.Simulation;
import star.meshing.MeshOperationManager;
import star.meshing.MeshPart;
import star.meshing.PartImportManager;
import star.meshing.RotationControl;
import star.meshing.TransformPartsOperation;
import star.meshing.TranslationControl;
import star.common.StarMacro;

/**
 * Stage 1: import STEP geometry, apply the DP18 machine transform, export each
 * transformed CAD body as a binary STL for reference labelling and bbox logic.
 */
public class PrepareGeometry extends StarMacro {
    private Properties props;

    private String p(String key) {
        String value = props.getProperty(key);
        if (value == null) {
            throw new IllegalArgumentException("Missing property: " + key);
        }
        return value;
    }

    private String p(String key, String fallback) {
        return props.getProperty(key, fallback);
    }

    private double pd(String key) {
        return Double.parseDouble(p(key));
    }

    private int pi(String key) {
        return Integer.parseInt(p(key));
    }

    private boolean pb(String key) {
        return Boolean.parseBoolean(p(key));
    }

    private String[] plist(String key) {
        String raw = p(key, "");
        if (raw.trim().isEmpty()) {
            return new String[0];
        }
        return raw.split("\\|", -1);
    }

    private void loadProperties() throws IOException {
        String path = System.getenv("STAR_WRAP_PARAMS");
        if (path == null || path.isBlank()) {
            throw new IllegalStateException("STAR_WRAP_PARAMS is not set");
        }
        props = new Properties();
        try (BufferedReader reader = Files.newBufferedReader(Paths.get(path), StandardCharsets.UTF_8)) {
            props.load(reader);
        }
    }

    private static String sanitize(String value) {
        String text = value == null ? "part" : value;
        text = text.replaceAll("[^A-Za-z0-9_.-]+", "_");
        text = text.replaceAll("_+", "_");
        text = text.replaceAll("^_+|_+$", "");
        return text.isEmpty() ? "part" : text;
    }

    private static String stem(String path) {
        String name = Paths.get(path).getFileName().toString();
        int dot = name.lastIndexOf('.');
        return dot > 0 ? name.substring(0, dot) : name;
    }

    private static boolean globMatch(String value, String pattern) {
        if (value == null) {
            return false;
        }
        StringBuilder regex = new StringBuilder("^");
        for (int i = 0; i < pattern.length(); i++) {
            char c = pattern.charAt(i);
            if (c == '*') {
                regex.append(".*");
            } else if (c == '?') {
                regex.append('.');
            } else {
                if ("\\.[]{}()+-^$|".indexOf(c) >= 0) {
                    regex.append('\\');
                }
                regex.append(c);
            }
        }
        regex.append('$');
        return Pattern.compile(regex.toString(), Pattern.CASE_INSENSITIVE).matcher(value).matches();
    }

    private static boolean matchesAny(String value, String[] patterns) {
        for (String pattern : patterns) {
            if (globMatch(value, pattern)) {
                return true;
            }
        }
        return false;
    }

    private static Set<Long> ids(Collection<GeometryPart> parts) {
        Set<Long> out = new HashSet<Long>();
        for (GeometryPart part : parts) {
            out.add(part.getObjectId());
        }
        return out;
    }

    private static List<GeometryPart> newParts(Simulation sim, Set<Long> before) {
        List<GeometryPart> out = new ArrayList<GeometryPart>();
        for (GeometryPart part : sim.getGeometryPartManager().getLeafParts()) {
            if (!before.contains(part.getObjectId())) {
                out.add(part);
            }
        }
        return out;
    }

    private static String parentName(GeometryPart part) {
        try {
            GeometryPart parent = part.getParentPart();
            return parent == null ? "" : parent.getPresentationName();
        } catch (Exception ignored) {
            return "";
        }
    }

    private static void addRotation(TransformPartsOperation op, double ox, double oy, double oz,
                                    double ax, double ay, double az, double degrees,
                                    star.common.Units angleUnits, int index) {
        RotationControl control = op.getTransforms().createRotationControl();
        control.setIndex(index);
        control.getOriginInput().setVector(new DoubleVector(new double[] {ox, oy, oz}));
        control.getAxisVectorInput().setVector(new DoubleVector(new double[] {ax, ay, az}));
        control.getAngle().setValueAndUnits(degrees, angleUnits);
    }

    private static void addTranslation(TransformPartsOperation op, double dx, double dy, double dz, int index) {
        TranslationControl control = op.getTransforms().createTranslationControl();
        control.setIndex(index);
        control.getTranslationVectorInput().setVector(new DoubleVector(new double[] {dx, dy, dz}));
    }

    private void applyMachine(Simulation sim, List<GeometryPart> parts, String sourceFile,
                              List<String> partNames) {
        if (!pb("machine_enabled") || parts.isEmpty()) {
            return;
        }

        boolean fixed = false;
        String[] fixedPatterns = plist("fixed_patterns");
        for (String name : partNames) {
            if (matchesAny(name, fixedPatterns)) {
                fixed = true;
                break;
            }
        }
        if (!fixed && matchesAny(stem(sourceFile), fixedPatterns)) {
            fixed = true;
        }

        MeshOperationManager manager = sim.get(MeshOperationManager.class);
        TransformPartsOperation op =
            (TransformPartsOperation) manager.createTransformPartsOperation(parts);
        op.setCreateNewOutputParts(false);

        star.common.Units deg = sim.getUnitsManager().getUnits("deg");
        int index = 0;
        addRotation(op, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 180.0, deg, index++);
        addTranslation(op, pd("machine_xpitch"), 0.0, 0.0, index++);

        if (!fixed) {
            double pitch = pd("machine_pitch_deg");
            if (Math.abs(pitch) > 1.0e-12) {
                addRotation(op, pd("machine_xpitch"), 0.0, pd("machine_zbase"),
                            0.0, 1.0, 0.0, pd("machine_pitch_sign") * pitch, deg, index++);
            }
            double dz = pd("machine_zpitch");
            if (Math.abs(dz) > 1.0e-12) {
                addTranslation(op, 0.0, 0.0, dz, index++);
            }
        }

        double yaw = pd("machine_yaw_deg");
        if (Math.abs(yaw) > 1.0e-12) {
            addRotation(op, pd("machine_yaw_pivot_x"), pd("machine_yaw_pivot_y"), 0.0,
                        0.0, 0.0, 1.0, pd("machine_yaw_sign") * yaw, deg, index++);
        }
        op.execute();
    }

    private void importAndExport(Simulation sim, PartImportManager importer,
                                 String sourceFile, String prefix, String exportDir,
                                 BufferedWriter index, int tessDensity, double sharpAngle) throws IOException {
        Set<Long> before = ids(sim.getGeometryPartManager().getLeafParts());
        importer.importCadPart(sourceFile, "SharpEdges", sharpAngle, tessDensity);
        List<GeometryPart> imported = newParts(sim, before);
        if (imported.isEmpty()) {
            throw new IllegalStateException("No geometry parts imported from: " + sourceFile);
        }

        List<String> originalNames = new ArrayList<String>();
        for (GeometryPart part : imported) {
            originalNames.add(part.getPresentationName() + " " + parentName(part));
        }
        applyMachine(sim, imported, sourceFile, originalNames);

        int seq = 0;
        for (GeometryPart part : imported) {
            String original = part.getPresentationName();
            String parent = parentName(part);
            String unique = prefix + "__" + sanitize(stem(sourceFile)) + "__" +
                String.format("%03d", seq) + "__" + sanitize(original);
            part.setPresentationName(unique);
            Path stl = Paths.get(exportDir).resolve(unique + ".stl");
            ((MeshPart) part).exportStlPart(stl.toString(), true);
            index.write(sourceFile);
            index.write('\t');
            index.write(original);
            index.write('\t');
            index.write(parent);
            index.write('\t');
            index.write(stl.toString());
            index.newLine();
            seq++;
        }
        System.out.println("Imported " + imported.size() + " parts from " + sourceFile);
    }

    @Override
    public void execute() {
        try {
            loadProperties();
            Simulation sim = getActiveSimulation();
            sim.loadMeshing();
            sim.loadCaeImport();
            PartImportManager importer = sim.get(PartImportManager.class);

            Path partsDir = Paths.get(p("parts_dir"));
            Path markersDir = Paths.get(p("markers_dir"));
            Files.createDirectories(partsDir);
            Files.createDirectories(markersDir);

            int tessDensity = pi("tessellation_density");
            double sharpAngle = pd("sharp_edge_angle_deg");

            try (BufferedWriter index = Files.newBufferedWriter(
                    Paths.get(p("parts_index")), StandardCharsets.UTF_8)) {
                for (String file : plist("car_files")) {
                    importAndExport(sim, importer, file, "CAR", partsDir.toString(),
                                    index, tessDensity, sharpAngle);
                }
            }

            String[] markerNames = plist("marker_names");
            String[] markerFiles = plist("marker_files");
            if (markerNames.length != markerFiles.length) {
                throw new IllegalStateException("marker_names and marker_files length mismatch");
            }
            try (BufferedWriter index = Files.newBufferedWriter(
                    Paths.get(p("markers_index")), StandardCharsets.UTF_8)) {
                for (int i = 0; i < markerFiles.length; i++) {
                    String prefix = "MARKER__" + sanitize(markerNames[i]);
                    importAndExport(sim, importer, markerFiles[i], prefix, markersDir.toString(),
                                    index, tessDensity, sharpAngle);
                }
            }

            sim.saveState(p("prepared_sim"));
            System.out.println("Prepared simulation saved: " + p("prepared_sim"));
        } catch (Exception ex) {
            ex.printStackTrace();
            throw new RuntimeException("PrepareGeometry failed", ex);
        }
    }
}
