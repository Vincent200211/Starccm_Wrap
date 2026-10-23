import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.Properties;
import java.util.regex.Pattern;

import star.common.GeometryPart;
import star.common.PartSurface;
import star.common.Simulation;
import star.meshing.BaseSize;
import star.meshing.CustomMeshControlManager;
import star.meshing.MeshOperationManager;
import star.meshing.MeshPart;
import star.meshing.PartImportManager;
import star.meshing.PartsMinimumSurfaceSize;
import star.meshing.SurfaceCustomMeshControl;
import star.meshing.PartsTargetSurfaceSize;
import star.meshing.PartsTargetSurfaceSizeOption;
import star.meshing.PartsMinimumSurfaceSizeOption;
import star.surfacewrapper.GlobalVolumeOfInterest;
import star.surfacewrapper.GlobalVolumeOfInterestOption;
import star.surfacewrapper.PartsTwoGroupContactPreventionSet;
import star.surfacewrapper.SmallestRelevantVolume;
import star.surfacewrapper.SurfaceWrapperAutoMesher;
import star.surfacewrapper.SurfaceWrapperAutoMeshOperation;
import star.common.StarMacro;

/**
 * Stage 2: build one global surface wrapper.
 *
 * - car/plinth surfaces: 2 mm
 * - inlet/outlet/sides/top: 1 m
 * - ground: nested planar bands growing from 2 mm to 1 m
 * - plinth and all ground bands: two-group contact prevention
 */
public class RunWrapper extends StarMacro {
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

    private static List<GeometryPart> partsByPrefix(Simulation sim, String prefix) {
        List<GeometryPart> out = new ArrayList<GeometryPart>();
        for (GeometryPart part : sim.getGeometryPartManager().getLeafParts()) {
            if (part.getPresentationName().startsWith(prefix)) {
                out.add(part);
            }
        }
        return out;
    }


    private static Set<Long> partIds(Simulation sim) {
        Set<Long> ids = new HashSet<Long>();
        for (GeometryPart part : sim.getGeometryPartManager().getLeafParts()) {
            ids.add(part.getObjectId());
        }
        return ids;
    }

    private static GeometryPart importStlNamed(PartImportManager importer, Simulation sim,
                                               String file, String name,
                                               star.common.Units metre) {
        Set<Long> before = partIds(sim);
        importer.importStlPart(file, "OneSurfaceForAllPatches", metre, false, 1.0e-5);
        List<GeometryPart> imported = new ArrayList<GeometryPart>();
        for (GeometryPart part : sim.getGeometryPartManager().getLeafParts()) {
            if (!before.contains(part.getObjectId())) {
                imported.add(part);
            }
        }
        if (imported.size() != 1) {
            throw new IllegalStateException("Expected one imported STL part from " + file +
                                            ", got " + imported.size());
        }
        GeometryPart part = imported.get(0);
        part.setPresentationName(name);
        return part;
    }

    private static void addSurfaceSize(CustomMeshControlManager controls, String name,
                                       List<GeometryPart> parts, double size,
                                       star.common.Units metre) {
        SurfaceCustomMeshControl control = controls.createSurfaceControl();
        control.setPresentationName(name);
        control.getGeometryObjects().setObjects(parts, true);
        control.getCustomConditions().get(PartsTargetSurfaceSizeOption.class)
            .setSelected(PartsTargetSurfaceSizeOption.Type.CUSTOM);
        control.getCustomConditions().get(PartsMinimumSurfaceSizeOption.class)
            .setSelected(PartsMinimumSurfaceSizeOption.Type.CUSTOM);
        control.getCustomValues().get(PartsTargetSurfaceSize.class)
            .setAbsoluteSize(size, metre);
        control.getCustomValues().get(PartsMinimumSurfaceSize.class)
            .setAbsoluteSize(0.5 * size, metre);
    }

    private static List<PartSurface> surfacesOf(List<GeometryPart> parts) {
        List<PartSurface> surfaces = new ArrayList<PartSurface>();
        for (GeometryPart part : parts) {
            surfaces.addAll(part.getPartSurfaces());
        }
        return surfaces;
    }

    private void addContactPrevention(SurfaceWrapperAutoMeshOperation op,
                                      List<GeometryPart> plinthParts,
                                      List<GeometryPart> groundParts,
                                      star.common.Units metre) {
        if (!pb("contact_prevention_enabled")) {
            return;
        }
        if (plinthParts.isEmpty()) {
            throw new IllegalStateException("No plinth parts found for contact prevention");
        }
        List<PartSurface> plinthSurfaces = surfacesOf(plinthParts);
        List<PartSurface> groundSurfaces = surfacesOf(groundParts);
        if (plinthSurfaces.isEmpty() || groundSurfaces.isEmpty()) {
            throw new IllegalStateException("Contact prevention surface set is empty");
        }
        PartsTwoGroupContactPreventionSet contact =
            op.getContactPreventionSet().createPartsTwoGroupContactPreventionSet();
        contact.setPresentationName("Plinth-ground contact prevention");
        contact.setEnableContactPrevention(true);
        contact.getFloor().setValueAndUnits(pd("contact_floor_m"), metre);
        contact.getPartSurfaceGroup1().setObjects(plinthSurfaces, true);
        contact.getPartSurfaceGroup2().setObjects(groundSurfaces, true);
        System.out.println("Contact prevention: " + plinthSurfaces.size() +
                           " plinth surfaces <-> " + groundSurfaces.size() + " ground surfaces");
    }

    @Override
    public void execute() {
        try {
            loadProperties();
            Simulation sim = getActiveSimulation();
            sim.loadMeshing();
            sim.loadCaeImport();

            PartImportManager importer = sim.get(PartImportManager.class);
            star.common.Units metre = sim.getUnitsManager().getUnits("m");

            String[] wallNames = plist("wall_names");
            String[] wallFiles = plist("wall_files");
            if (wallNames.length != wallFiles.length) {
                throw new IllegalStateException("wall_names and wall_files length mismatch");
            }
            List<GeometryPart> wallParts = new ArrayList<GeometryPart>();
            for (int i = 0; i < wallFiles.length; i++) {
                wallParts.add(importStlNamed(importer, sim, wallFiles[i], wallNames[i], metre));
            }

            String[] groundNames = plist("ground_names");
            String[] groundFiles = plist("ground_files");
            String[] groundSizes = plist("ground_sizes_m");
            if (groundNames.length != groundFiles.length || groundNames.length != groundSizes.length) {
                throw new IllegalStateException("ground name/file/size lists do not match");
            }
            List<GeometryPart> groundParts = new ArrayList<GeometryPart>();
            for (int i = 0; i < groundFiles.length; i++) {
                groundParts.add(importStlNamed(importer, sim, groundFiles[i], groundNames[i], metre));
            }

            List<GeometryPart> cars = partsByPrefix(sim, "CAR__");
            System.out.println("CHECKPOINT car parts=" + cars.size());
            if (cars.isEmpty()) {
                throw new IllegalStateException("No CAR__ parts found in prepared simulation");
            }

            List<GeometryPart> domainParts = new ArrayList<GeometryPart>();
            domainParts.addAll(wallParts);
            domainParts.addAll(groundParts);
            Collection<GeometryPart> wrapperInputs = new ArrayList<GeometryPart>(cars);
            wrapperInputs.addAll(domainParts);

            System.out.println("CHECKPOINT before wrapper operation creation");
            MeshOperationManager manager = sim.get(MeshOperationManager.class);
            SurfaceWrapperAutoMeshOperation op = (SurfaceWrapperAutoMeshOperation)
                manager.createSurfaceWrapperAutoMeshOperation(wrapperInputs, p("wrap_name"));
            op.setMeshPartByPart(false);
            op.setMeshInParallel(pb("mesh_in_parallel"));
            op.setDoPartialWrapping(false);
            op.setRunGlobalIfLocalFails(pb("run_global_if_local_fails"));
            op.setCreateContacts(false);
            System.out.println("CHECKPOINT wrapper operation created");

            op.getDefaultValues().get(BaseSize.class).setValue(pd("base_size_m"));
            op.getDefaultValues().get(PartsMinimumSurfaceSize.class)
                .setAbsoluteSize(pd("minimum_surface_size_m"), metre);

            GlobalVolumeOfInterest voi = op.getDefaultValues().get(GlobalVolumeOfInterest.class);
            voi.getVolumeOfInterestOption().setSelected(
                GlobalVolumeOfInterestOption.Type.LARGEST_INTERNAL);
            SmallestRelevantVolume smallest = voi.getSmallestRelevantVolume();
            smallest.setValue(pd("smallest_relevant_volume_m3"));
            System.out.println("CHECKPOINT global wrapper controls set");

            CustomMeshControlManager controls = op.getCustomMeshControls();
            System.out.println("CHECKPOINT adding surface controls");
            addSurfaceSize(controls, "Car and plinth 2 mm", cars,
                           pd("car_target_size_m"), metre);
            addSurfaceSize(controls, "Wind-tunnel walls 1 m", wallParts,
                           pd("wall_target_size_m"), metre);
            for (int i = 0; i < groundParts.size(); i++) {
                addSurfaceSize(controls, "Ground band " + i,
                               Collections.singletonList(groundParts.get(i)),
                               Double.parseDouble(groundSizes[i]), metre);
            }

            String plinthPattern = p("contact_prevention_plinth_pattern", "*Plinth*");
            List<GeometryPart> plinthParts = new ArrayList<GeometryPart>();
            for (GeometryPart part : cars) {
                String text = part.getPresentationName();
                if (globMatch(text, plinthPattern)) {
                    plinthParts.add(part);
                }
            }
            System.out.println("CHECKPOINT before contact prevention, plinthParts=" + plinthParts.size());
            List<GeometryPart> contactGroundParts =
                Collections.singletonList(groundParts.get(0));
            addContactPrevention(op, plinthParts, contactGroundParts, metre);
            System.out.println("CHECKPOINT contact prevention added");

            SurfaceWrapperAutoMesher mesher = op.getSurfaceWrapperAutoMesher();
            mesher.setDoCurvatureRefinement(pb("curvature_refinement"));
            mesher.setDoProximityRefinement(pb("proximity_refinement"));
            mesher.setDoGapClosure(pb("gap_closure"));
            mesher.setRunGlobalIfLocalFails(pb("run_global_if_local_fails"));

            System.out.println("CHECKPOINT all wrapper controls set");
            System.out.println("Executing surface wrapper: " + p("wrap_name"));
            op.execute();

            GeometryPart output = sim.getGeometryPartManager().getPart(p("wrap_name"));
            if (!(output instanceof MeshPart)) {
                throw new IllegalStateException("Wrapper output is not a MeshPart: " + output);
            }
            ((MeshPart) output).exportStlPart(p("wrap_stl"), pb("stl_binary"));
            sim.saveState(p("wrap_sim"));
            System.out.println("Wrapper STL: " + p("wrap_stl"));
            System.out.println("Wrapper simulation: " + p("wrap_sim"));
        } catch (Exception ex) {
            System.out.println("WRAPPER FAILURE: " + ex.getClass().getName() + ": " + ex.getMessage());
            ex.printStackTrace(System.out);
            Throwable cause = ex.getCause();
            while (cause != null) {
                System.out.println("WRAPPER CAUSE: " + cause.getClass().getName() + ": " + cause.getMessage());
                cause.printStackTrace(System.out);
                cause = cause.getCause();
            }
            throw new RuntimeException("RunWrapper failed", ex);
        }
    }
}
