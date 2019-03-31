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
    public static Pose2d START_DEPOT = new Pose2d(-START_DIST, START_DIST,  3 * PI / 4);
    public static double LEFT_SECOND_DEPOT = 43;
    public static double LEFT_SECOND_CRATER = 20.5;
    public static int LEFT_SECOND_EXTEND = 0;
    public static double CENTER_SECOND_DEPOT = 37;
    public static double CENTER_SECOND_CRATER = -9;
    public static int CENTER_SECOND_EXTEND = -300;
    public static double RIGHT_SECOND_DEPOT = 22;
    public static double RIGHT_SECOND_CRATER = -36;
    public static int RIGHT_SECOND_EXTEND = -700;
    public static double LEFT_DEPOT = 37;
    public static double CENTER_DEPOT = 0;
    public static double RIGHT_DEPOT = -37;
    public static Pose2d CRATER_SINGLE_INTERMEDIARY = new Pose2d(10, 40, 110.0 * (PI / 180));
    public static Pose2d CRATER_SINGLE = new Pose2d(-10, 58, 180.0 * (PI / 180));
    public static Pose2d CRATER_DOUBLE = new Pose2d(-2, 50, 165.0 * (PI / 180));
    public static Pose2d CRATER_COLLECT = new Pose2d(16, 20, PI / 4);
    public static Pose2d CRATER_DUMP = new Pose2d(14, 21, 40.0 * (PI / 180));
    public static Pose2d DEPOT_PARK = new Pose2d(-46, 0, 240 * (PI / 180));

}
