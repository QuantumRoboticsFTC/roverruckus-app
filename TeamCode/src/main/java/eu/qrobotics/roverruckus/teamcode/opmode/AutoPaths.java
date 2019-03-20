package eu.qrobotics.roverruckus.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class AutoPaths {
    public enum FieldLocation {
        CRATER,
        DEPOT
    }

    private static double PI = Math.PI;
    public static double START_DIST = 20.5 / Math.sqrt(2);
    public static Pose2d START_CRATER = new Pose2d(START_DIST, START_DIST, PI / 4);
    public static double LEFT_SECOND_DEPOT = 43;
    public static double LEFT_SECOND_CRATER = 20;
    public static Pose2d CRATER_SECOND = new Pose2d(0, 50, 165.0 * (PI / 180));
    public static Pose2d CRATER_COLLECT = new Pose2d(16, 20, PI / 4);
    public static Pose2d CRATER_DUMP = new Pose2d(13, 20, PI / 4);
}
