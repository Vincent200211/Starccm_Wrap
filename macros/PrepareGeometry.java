import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Properties;
import java.util.Set;
import java.util.regex.Pattern;

import star.base.neo.DoubleVector;
import star.common.CartesianCoordinateSystem;
import star.common.CoordinateSystem;
import star.common.GeometryPart;
import star.common.GeometryPartManager;
import star.common.Simulation;
import star.common.StarMacro;
import star.common.Units;
import star.meshing.MeshPart;
import star.meshing.PartImportManager;

/**
 * Import STEP files, assign stable file-based part names, flatten the assembly
 * leaves to the root part manager, then apply the global machine motion:
 *   1. all parts: Rz(180) + Tx(xpitch)
 *   2. lifted parts only: pitch + dz
 *   3. all parts: yaw
 *
 * GeometryPartManager transform angles are in radians.
 */
public class PrepareGeometry extends StarMacro {
    private Properties props;

    private static class ImportedRecord {
        String sourceFile;
        List<GeometryPart> parts = new ArrayList<GeometryPart>();
        List<String> originalNames = new ArrayList<String>();
        List<String> parentNames = new ArrayList<String>();
        List<String> uniqueNames = new ArrayList<String>();
    }

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

    private static List<Double> vector(double x, double y, double z) {
        return Arrays.asList(x, y, z);
    }

    private static List<Units> vectorUnits(Units unit) {
        return Arrays.asList(unit, unit, unit);
    }

    private static CartesianCoordinateSystem createPivotSystem(
            Simulation sim, String name, double x, double y, double z, Units metre) {
        CartesianCoordinateSystem csys = sim.getCoordinateSystemManager()
            .createCoordinateSystem(CartesianCoordinateSystem.class, name, false);
        csys.setOrigin(new DoubleVector(new double[] {x, y, z}),
                       new Units[] {metre, metre, metre});
        csys.setBasis0(new DoubleVector(new double[] {1.0, 0.0, 0.0}));
        csys.setBasis1(new DoubleVector(new double[] {0.0, 1.0, 0.0}));
        return csys;
    }

    private ImportedRecord importParts(Simulation sim, PartImportManager importer, String sourceFile,
                                       String prefix, int tessDensity, double sharpAngle) {
        Set<Long> before = ids(sim.getGeometryPartManager().getLeafParts());
        importer.importCadPart(sourceFile, "SharpEdges", sharpAngle, tessDensity);
        List<GeometryPart> imported = newParts(sim, before);
        if (imported.isEmpty()) {
            throw new IllegalStateException("No geometry parts imported from: " + sourceFile);
        }
        ImportedRecord record = new ImportedRecord();
        record.sourceFile = sourceFile;
        for (int i = 0; i < imported.size(); i++) {
            GeometryPart part = imported.get(i);
            record.parts.add(part);
            record.originalNames.add(part.getPresentationName());
            record.parentNames.add(parentName(part));
            String unique = prefix + "__" + String.format("%03d", i);
            part.setPresentationName(unique);
            record.uniqueNames.add(unique);
        }
        System.out.println("Imported " + imported.size() + " parts from " + sourceFile);
        return record;
    }

    private static GeometryPart findCurrentPart(Simulation sim, String name) {
        for (GeometryPart part : sim.getGeometryPartManager().getLeafParts()) {
            if (name.equals(part.getPresentationName())) {
                return part;
            }
        }
        throw new IllegalStateException("Current geometry part not found: " + name);
    }

    private static List<GeometryPart> partsByUniqueNames(Simulation sim, List<String> names) {
        List<GeometryPart> out = new ArrayList<GeometryPart>();
        for (String name : names) {
            out.add(findCurrentPart(sim, name));
        }
        return out;
    }

    private boolean recordIsFixed(ImportedRecord record) {
        String[] fixedPatterns = plist("fixed_patterns");
        for (int i = 0; i < record.parts.size(); i++) {
            String text = record.originalNames.get(i) + " " + record.parentNames.get(i);
            if (matchesAny(text, fixedPatterns)) {
                return true;
            }
        }
        return matchesAny(stem(record.sourceFile), fixedPatterns);
    }

    private static void writeRecord(Simulation sim, ImportedRecord record, String exportDir,
                                    BufferedWriter index) throws IOException {
        for (int i = 0; i < record.parts.size(); i++) {
            String unique = record.uniqueNames.get(i);
            GeometryPart part = findCurrentPart(sim, unique);
            Path stl = Paths.get(exportDir).resolve(unique + ".stl");
            ((MeshPart) part).exportStlPart(stl.toString(), true);
            index.write(record.sourceFile);
            index.write('\t');
            index.write(record.originalNames.get(i));
            index.write('\t');
            index.write(record.parentNames.get(i));
            index.write('\t');
            index.write(stl.toString());
            index.newLine();
        }
    }

    @Override
    public void execute() {
        try {
            loadProperties();
            Simulation sim = getActiveSimulation();
            sim.loadMeshing();
            sim.loadCaeImport();
            PartImportManager importer = sim.get(PartImportManager.class);
            GeometryPartManager partManager = sim.getGeometryPartManager();

            int tessDensity = pi("tessellation_density");
            double sharpAngle = pd("sharp_edge_angle_deg");

            List<ImportedRecord> carRecords = new ArrayList<ImportedRecord>();
            List<ImportedRecord> markerRecords = new ArrayList<ImportedRecord>();
            for (String file : plist("car_files")) {
                String prefix = "CAR__" + sanitize(stem(file));
                carRecords.add(importParts(sim, importer, file, prefix, tessDensity, sharpAngle));
            }
            String[] markerNames = plist("marker_names");
            String[] markerFiles = plist("marker_files");
            if (markerNames.length != markerFiles.length) {
                throw new IllegalStateException("marker_names and marker_files length mismatch");
            }
            for (int i = 0; i < markerFiles.length; i++) {
                String prefix = "MARKER__" + sanitize(markerNames[i]) + "__" +
                    sanitize(stem(markerFiles[i]));
                markerRecords.add(importParts(sim, importer, markerFiles[i], prefix,
                                              tessDensity, sharpAngle));
            }

            List<ImportedRecord> allRecords = new ArrayList<ImportedRecord>();
            allRecords.addAll(carRecords);
            allRecords.addAll(markerRecords);
            List<String> allNames = new ArrayList<String>();
            List<String> liftedNames = new ArrayList<String>();
            for (ImportedRecord record : allRecords) {
                allNames.addAll(record.uniqueNames);
                if (!recordIsFixed(record)) {
                    liftedNames.addAll(record.uniqueNames);
                }
            }

            // Flatten imported assembly leaves onto the root part manager so all
            // subsequent coordinate transforms use the laboratory frame.
            partManager.reparentParts(partsByUniqueNames(sim, allNames), partManager);
            partsByUniqueNames(sim, allNames);

            if (pb("machine_enabled") && !allNames.isEmpty()) {
                Units metre = sim.getUnitsManager().getUnits("m");
                CoordinateSystem labCsys =
                    sim.getCoordinateSystemManager().getLabCoordinateSystem();

                partManager.rotateParts(partsByUniqueNames(sim, allNames),
                                        vector(0.0, 0.0, 1.0), vectorUnits(metre),
                                        Math.PI, labCsys);
                partManager.translateParts(partsByUniqueNames(sim, allNames),
                                           vector(pd("machine_xpitch"), 0.0, 0.0),
                                           vectorUnits(metre), labCsys);

                double pitch = pd("machine_pitch_deg");
                double dz = pd("machine_zpitch");
                boolean applyPitch = pb("machine_apply_pitch") && Math.abs(pitch) > 1.0e-12;
                boolean applyDz = pb("machine_apply_dz") && Math.abs(dz) > 1.0e-12;
                if ((applyPitch || applyDz) && !liftedNames.isEmpty()) {
                    if (applyPitch) {
                        CartesianCoordinateSystem pitchCsys = createPivotSystem(
                            sim, "Pitch Pivot", pd("machine_xpitch"), 0.0,
                            pd("machine_zbase"), metre);
                        partManager.rotateParts(partsByUniqueNames(sim, liftedNames),
                                                vector(0.0, 1.0, 0.0), vectorUnits(metre),
                                                Math.toRadians(pd("machine_pitch_sign") * pitch),
                                                pitchCsys);
                    }
                    if (applyDz) {
                        partManager.translateParts(partsByUniqueNames(sim, liftedNames),
                                                   vector(0.0, 0.0, dz),
                                                   vectorUnits(metre), labCsys);
                    }
                }

                double yaw = pd("machine_yaw_deg");
                if (Math.abs(yaw) > 1.0e-12) {
                    CartesianCoordinateSystem yawCsys = createPivotSystem(
                        sim, "Yaw Pivot", pd("machine_yaw_pivot_x"),
                        pd("machine_yaw_pivot_y"), 0.0, metre);
                    partManager.rotateParts(partsByUniqueNames(sim, allNames),
                                            vector(0.0, 0.0, 1.0), vectorUnits(metre),
                                            Math.toRadians(pd("machine_yaw_sign") * yaw),
                                            yawCsys);
                }

                partsByUniqueNames(sim, allNames);
                System.out.println("Applied machine transform to all " + allNames.size() +
                                   " parts after flattening");
            }

            try (BufferedWriter index = Files.newBufferedWriter(
                    Paths.get(p("parts_index")), StandardCharsets.UTF_8)) {
                for (ImportedRecord record : carRecords) {
                    writeRecord(sim, record, p("parts_dir"), index);
                }
            }
            try (BufferedWriter index = Files.newBufferedWriter(
                    Paths.get(p("markers_index")), StandardCharsets.UTF_8)) {
                for (ImportedRecord record : markerRecords) {
                    writeRecord(sim, record, p("markers_dir"), index);
                }
            }

            sim.saveState(p("prepared_sim"));
            System.out.println("Prepared simulation saved: " + p("prepared_sim"));
        } catch (Exception ex) {
            System.out.println("PREPARE FAILURE: " + ex.getClass().getName() + ": " + ex.getMessage());
            ex.printStackTrace(System.out);
            throw new RuntimeException("PrepareGeometry failed", ex);
        }
    }
}
