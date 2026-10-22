import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Properties;
import java.util.regex.Pattern;

import star.common.GeometryPart;
import star.common.Simulation;
import star.meshing.BaseSize;
import star.meshing.CustomMeshControlManager;
import star.meshing.MeshOperationManager;
import star.meshing.MeshPart;
import star.meshing.PartImportManager;
import star.meshing.PartsMinimumSurfaceSize;
import star.meshing.PartsTargetSurfaceSize;
import star.meshing.SurfaceCustomMeshControl;
import star.meshing.VolumeControlSize;
import star.meshing.VolumeCustomMeshControl;
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

    private static void addSurfaceSize(CustomMeshControlManager controls, String name,
                                       List<GeometryPart> parts, double size,
                                       star.common.Units metre) {
        SurfaceCustomMeshControl control = controls.createSurfaceControl();
        control.setPresentationName(name);
        control.getGeometryObjects().addParts(parts);
        control.getCustomValues().get(PartsTargetSurfaceSize.class)
            .setAbsoluteSize(size, metre);
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
        PartsTwoGroupContactPreventionSet contact =
            op.getContactPreventionSet().createPartsTwoGroupContactPreventionSet();
        contact.setPresentationName("Plinth-ground contact prevention");
        contact.setEnableContactPrevention(true);
        contact.getFloor().setValueAndUnits(pd("contact_floor_m"), metre);

        List<GeometryPart> all = new ArrayList<GeometryPart>();
        all.addAll(plinthParts);
        all.addAll(groundParts);
        contact.setParts(all);
        contact.getPartSurfaceGroup1().addParts(plinthParts);
        contact.getPartSurfaceGroup2().addParts(groundParts);
        System.out.println("Contact prevention: plinth surfaces <-> " +
                           groundParts.size() + " ground parts");
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
                importer.importStlPart(wallFiles[i], wallNames[i], metre, false, 1.0e-5);
                wallParts.add(sim.getGeometryPartManager().getPart(wallNames[i]));
            }

            String[] groundNames = plist("ground_names");
            String[] groundFiles = plist("ground_files");
            String[] groundSizes = plist("ground_sizes_m");
            if (groundNames.length != groundFiles.length || groundNames.length != groundSizes.length) {
                throw new IllegalStateException("ground name/file/size lists do not match");
            }
            List<GeometryPart> groundParts = new ArrayList<GeometryPart>();
            for (int i = 0; i < groundFiles.length; i++) {
                importer.importStlPart(groundFiles[i], groundNames[i], metre, false, 1.0e-5);
                groundParts.add(sim.getGeometryPartManager().getPart(groundNames[i]));
            }

            importer.importStlPart(p("refine_box_stl"), "CarRefineBox", metre, false, 1.0e-5);
            GeometryPart refineBox = sim.getGeometryPartManager().getPart("CarRefineBox");
            List<GeometryPart> cars = partsByPrefix(sim, "CAR__");
            if (cars.isEmpty()) {
                throw new IllegalStateException("No CAR__ parts found in prepared simulation");
            }

            List<GeometryPart> domainParts = new ArrayList<GeometryPart>();
            domainParts.addAll(wallParts);
            domainParts.addAll(groundParts);
            Collection<GeometryPart> wrapperInputs = new ArrayList<GeometryPart>(cars);
            wrapperInputs.addAll(domainParts);

            MeshOperationManager manager = sim.get(MeshOperationManager.class);
            SurfaceWrapperAutoMeshOperation op = (SurfaceWrapperAutoMeshOperation)
                manager.createSurfaceWrapperAutoMeshOperation(wrapperInputs, p("wrap_name"));
            op.setMeshPartByPart(false);
            op.setDoPartialWrapping(false);
            op.setRunGlobalIfLocalFails(pb("run_global_if_local_fails"));
            op.setCreateContacts(false);

            op.getDefaultValues().get(BaseSize.class).setValue(pd("base_size_m"));
            op.getDefaultValues().get(PartsMinimumSurfaceSize.class)
                .setAbsoluteSize(pd("minimum_surface_size_m"), metre);

            GlobalVolumeOfInterest voi = op.getDefaultValues().get(GlobalVolumeOfInterest.class);
            voi.getVolumeOfInterestOption().setSelected(GlobalVolumeOfInterestOption.Type.NTH_LARGEST);
            voi.getVolumeOfInterestVolumeNumber().setVolumeNumber(Integer.parseInt(p("nth_largest")));
            SmallestRelevantVolume smallest = op.getDefaultValues().get(SmallestRelevantVolume.class);
            smallest.setValue(pd("smallest_relevant_volume_m3"));

            CustomMeshControlManager controls = op.getCustomMeshControls();
            addSurfaceSize(controls, "Car and plinth 2 mm", cars,
                           pd("car_target_size_m"), metre);
            addSurfaceSize(controls, "Wind-tunnel walls 1 m", wallParts,
                           pd("wall_target_size_m"), metre);
            for (int i = 0; i < groundParts.size(); i++) {
                addSurfaceSize(controls, "Ground band " + i,
                               Collections.singletonList(groundParts.get(i)),
                               Double.parseDouble(groundSizes[i]), metre);
            }

            VolumeCustomMeshControl refineControl = controls.createVolumeControl();
            refineControl.setPresentationName("Car refinement volume");
            refineControl.getGeometryObjects().addParts(Collections.singletonList(refineBox));
            refineControl.getCustomValues().get(VolumeControlSize.class)
                .setAbsoluteSize(pd("refine_volume_size_m"), metre);

            String plinthPattern = p("contact_prevention_plinth_pattern", "*Plinth*");
            List<GeometryPart> plinthParts = new ArrayList<GeometryPart>();
            for (GeometryPart part : cars) {
                String text = part.getPresentationName();
                if (globMatch(text, plinthPattern)) {
                    plinthParts.add(part);
                }
            }
            addContactPrevention(op, plinthParts, groundParts, metre);

            SurfaceWrapperAutoMesher mesher = op.getSurfaceWrapperAutoMesher();
            mesher.setDoCurvatureRefinement(pb("curvature_refinement"));
            mesher.setDoProximityRefinement(pb("proximity_refinement"));
            mesher.setDoGapClosure(pb("gap_closure"));
            mesher.setRunGlobalIfLocalFails(pb("run_global_if_local_fails"));

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
            ex.printStackTrace();
            throw new RuntimeException("RunWrapper failed", ex);
        }
    }
}
