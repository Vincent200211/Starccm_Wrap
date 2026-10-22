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
import star.surfacewrapper.SmallestRelevantVolume;
import star.surfacewrapper.SurfaceWrapperAutoMesher;
import star.surfacewrapper.SurfaceWrapperAutoMeshOperation;
import star.common.StarMacro;

/**
 * Stage 2: import the outer-domain/refinement boxes, build one global surface
 * wrapper with Nth-largest internal volume, variable sizing, and export STL.
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

    private double pd(String key) {
        return Double.parseDouble(p(key));
    }

    private boolean pb(String key) {
        return Boolean.parseBoolean(p(key));
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

    private static List<GeometryPart> carParts(Simulation sim) {
        List<GeometryPart> out = new ArrayList<GeometryPart>();
        for (GeometryPart part : sim.getGeometryPartManager().getLeafParts()) {
            if (part.getPresentationName().startsWith("CAR__")) {
                out.add(part);
            }
        }
        return out;
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
            importer.importStlPart(p("domain_box_stl"), "DomainBox", metre, false, 1.0e-5);
            importer.importStlPart(p("refine_box_stl"), "CarRefineBox", metre, false, 1.0e-5);

            GeometryPart domainBox = sim.getGeometryPartManager().getPart("DomainBox");
            GeometryPart refineBox = sim.getGeometryPartManager().getPart("CarRefineBox");
            List<GeometryPart> cars = carParts(sim);
            if (cars.isEmpty()) {
                throw new IllegalStateException("No CAR__ parts found in prepared simulation");
            }

            Collection<GeometryPart> wrapperInputs = new ArrayList<GeometryPart>(cars);
            wrapperInputs.add(domainBox);

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
            SurfaceCustomMeshControl carControl = controls.createSurfaceControl();
            carControl.setPresentationName("Car surface size");
            carControl.getGeometryObjects().addParts(cars);
            carControl.getCustomValues().get(PartsTargetSurfaceSize.class)
                .setAbsoluteSize(pd("car_target_size_m"), metre);
            try {
                carControl.getCustomValues().get(PartsMinimumSurfaceSize.class)
                    .setAbsoluteSize(pd("minimum_surface_size_m"), metre);
            } catch (Exception ignored) {
                // Older custom-control sets may not expose a separate minimum value.
            }

            VolumeCustomMeshControl refineControl = controls.createVolumeControl();
            refineControl.setPresentationName("Car refinement volume");
            refineControl.getGeometryObjects().addParts(Collections.singletonList(refineBox));
            refineControl.getCustomValues().get(VolumeControlSize.class)
                .setAbsoluteSize(pd("refine_volume_size_m"), metre);

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
