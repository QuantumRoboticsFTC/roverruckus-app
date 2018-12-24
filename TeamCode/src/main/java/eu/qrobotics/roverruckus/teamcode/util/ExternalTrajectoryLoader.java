package eu.qrobotics.roverruckus.teamcode.util;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.fasterxml.jackson.module.kotlin.KotlinModule;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

@Config
public class ExternalTrajectoryLoader {
    public static String DIR_NAME = "Quantum";
    private static final ObjectMapper MAPPER = new ObjectMapper(new YAMLFactory());

    static {
        MAPPER.registerModule(new KotlinModule((512)));
    }

    private static File getTrajectoryRoot() {
        File dir = new File(Environment.getExternalStorageDirectory(), DIR_NAME);
        dir.mkdirs();
        return dir;
    }

    /**
     * Loads a trajectory config with the given name.
     */
    public static TrajectoryConfig loadConfig(String name) throws IOException {
        InputStream inputStream = new FileInputStream(new File(getTrajectoryRoot(), name + ".yaml"));
        return MAPPER.readValue(inputStream, TrajectoryConfig.class);
    }

    /**
     * Loads a trajectory with the given name.
     *
     * @see #loadConfig(String)
     */
    public static Trajectory load(String name) throws IOException {
        return loadConfig(name).toTrajectory();
    }
}
