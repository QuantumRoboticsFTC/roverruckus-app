package eu.qrobotics.roverruckus.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import java.util.ArrayList;
import java.util.List;

import eu.qrobotics.roverruckus.teamcode.navigation.GoodLinearInterpolator;
import eu.qrobotics.roverruckus.teamcode.subsystems.DriveConstants;

@Config
public class AutoPaths {
    public enum FieldLocation {
        CRATER,
        DEPOT
    }

    private static double PI = Math.PI;
    public static double START_DIST = 20.5 / Math.sqrt(2);
    public static Pose2d START_CRATER = new Pose2d(START_DIST, START_DIST, PI / 4);
    public static Pose2d START_DEPOT = new Pose2d(-START_DIST, START_DIST, 3 * PI / 4);
    public static double LEFT_SECOND_DEPOT = 43;
    public static double LEFT_SECOND_CRATER = 20.5;
    public static int LEFT_SECOND_EXTEND = 0;
    public static double CENTER_SECOND_DEPOT = 37;
    public static double CENTER_SECOND_CRATER = -14;
    public static int CENTER_SECOND_EXTEND = -300;
    public static double RIGHT_SECOND_DEPOT = 22;
    public static double RIGHT_SECOND_CRATER = -37;
    public static int RIGHT_SECOND_EXTEND = -700;
    public static double LEFT_CRATER = 24;
    public static double CENTER_CRATER = -9;
    public static double RIGHT_CRATER = -37;
    public static double LEFT_DEPOT = 32;
    public static double CENTER_DEPOT = -9;
    public static double RIGHT_DEPOT = -42;
    public static Pose2d CRATER_SINGLE_INTERMEDIARY = new Pose2d(10, 40, 110.0 * (PI / 180));
    public static Pose2d CRATER_SINGLE = new Pose2d(-10, 58, 180.0 * (PI / 180));
    public static Pose2d CRATER_DOUBLE = new Pose2d(-2, 50, 165.0 * (PI / 180));
    public static Pose2d CRATER_COLLECT = new Pose2d(15, 19, PI / 4);
    public static Pose2d CRATER_DOUBLE_DUMP = new Pose2d(14, 21, 40.0 * (PI / 180));
    public static Pose2d CRATER_DUMP = new Pose2d(12, 19, 36.0 * (PI / 180));
    public static Vector2d CRATER_TAKE = new Vector2d(25.5, 30.5);
    public static Pose2d DEPOT_PARK = new Pose2d(-46, 0, 240 * (PI / 180));
    public static Pose2d DEPOT_DUMP = new Pose2d(-13, 17.5, 3 * PI / 4);
    public static Pose2d DEPOT_FIRST_CUBE_DUMP = new Pose2d(-14.5, 17.5, 3 * PI / 4 - 5 * (PI / 180));
    public static Vector2d DEPOT_TAKE = new Vector2d(-52.5, -11.26);
    public static List<Trajectory>[][] trajectories = new ArrayList[3][4];

    private static boolean isInited = false;

    public static void init() {
//        if (isInited)
//            return;
//        isInited = true;

        //<editor-fold desc="Depot Single Sample">
        for (int i = 1; i <= 3; i++) {
            double angle = AutoPaths.LEFT_DEPOT;
            switch (i) {
                case 2:
                    angle = AutoPaths.CENTER_DEPOT;
                    break;
                case 3:
                    angle = AutoPaths.RIGHT_DEPOT;
                    break;
            }

            trajectories[0][i] = new ArrayList<>();

            trajectories[0][i].add(
                    new TrajectoryBuilder(AutoPaths.START_DEPOT, DriveConstants.BASE_CONSTRAINTS)
                            .strafeLeft(4)
                            .build());

            trajectories[0][i].add(
                    new TrajectoryBuilder(trajectories[0][i].get(0).end(), DriveConstants.BASE_CONSTRAINTS)
                            .forward(10)
                            .build());
            trajectories[0][i].add(
                    new TrajectoryBuilder(trajectories[0][i].get(1).end(), DriveConstants.BASE_CONSTRAINTS)
                            .turn(angle * (Math.PI / 180))
                            .waitFor(0.2)
                            .build());
            trajectories[0][i].add(
                    new TrajectoryBuilder(trajectories[0][i].get(2).end(), DriveConstants.SPECIAL_CONSTRAINTS)
//                            .turn(-1 * angle * (Math.PI / 180))
//                            .splineTo(AutoPaths.DEPOT_DUMP)
                            .reverse()
                            .splineTo(AutoPaths.DEPOT_FIRST_CUBE_DUMP)
                            .waitFor(0.5)
                            .build());
            trajectories[0][i].add(
                    new TrajectoryBuilder(trajectories[0][i].get(3).end(), DriveConstants.BASE_CONSTRAINTS)
                            .splineTo(AutoPaths.DEPOT_PARK)
                            .build());
            trajectories[0][i].add(
                    new TrajectoryBuilder(AutoPaths.DEPOT_PARK, DriveConstants.BASE_CONSTRAINTS)
                            .lineTo(DEPOT_TAKE, new GoodLinearInterpolator(AutoPaths.DEPOT_PARK.getHeading(), AutoPaths.DEPOT_PARK.getHeading() + 10 * Math.PI / 180))
                            .build());
            trajectories[0][i].add(
                    new TrajectoryBuilder(trajectories[0][i].get(5).end(), DriveConstants.BASE_CONSTRAINTS)
                            .reverse()
                            .splineTo(AutoPaths.DEPOT_DUMP)
                            .waitFor(1)
                            .build());
            trajectories[0][i].add(
                    new TrajectoryBuilder(AutoPaths.DEPOT_DUMP, DriveConstants.BASE_CONSTRAINTS)
                            .strafeLeft(6)
                            .splineTo(AutoPaths.DEPOT_PARK)
                            .waitFor(0.2)
                            .build());
        }
        //</editor-fold>

        //<editor-fold desc="Crater Single Sample">
        for (int i = 1; i <= 3; i++) {
            double angle = AutoPaths.LEFT_CRATER;
            switch (i) {
                case 2:
                    angle = AutoPaths.CENTER_CRATER;
                    break;
                case 3:
                    angle = AutoPaths.RIGHT_CRATER;
                    break;
            }

            trajectories[1][i] = new ArrayList<>();

            trajectories[1][i].add(
                    new TrajectoryBuilder(AutoPaths.START_CRATER, DriveConstants.BASE_CONSTRAINTS)
                            .strafeLeft(4)
                            .build());

            trajectories[1][i].add(
                    new TrajectoryBuilder(trajectories[1][i].get(0).end(), DriveConstants.BASE_CONSTRAINTS)
                            .beginComposite()
                            .splineTo(AutoPaths.CRATER_SINGLE_INTERMEDIARY, new GoodLinearInterpolator(AutoPaths.START_CRATER.getHeading(), AutoPaths.CRATER_SINGLE_INTERMEDIARY.getHeading()))
                            .splineTo(AutoPaths.CRATER_SINGLE, new GoodLinearInterpolator(AutoPaths.CRATER_SINGLE_INTERMEDIARY.getHeading(), AutoPaths.CRATER_SINGLE.getHeading()))
                            .closeComposite()
                            .build());
            trajectories[1][i].add(
                    new TrajectoryBuilder(trajectories[1][i].get(1).end(), DriveConstants.BASE_CONSTRAINTS)
                            .reverse()
                            .beginComposite()
                            .splineTo(AutoPaths.CRATER_SINGLE_INTERMEDIARY)
                            .splineTo(AutoPaths.CRATER_COLLECT.plus(new Pose2d(0, 0, angle * Math.PI / 180)))
                            .closeComposite()
                            .waitFor(0.3)
                            .build());
            trajectories[1][i].add(
                    new TrajectoryBuilder(trajectories[1][i].get(2).end(), DriveConstants.BASE_CONSTRAINTS)
                            .reverse()
                            .splineTo(AutoPaths.CRATER_DUMP, new GoodLinearInterpolator(trajectories[1][i].get(2).end().getHeading(), AutoPaths.CRATER_DUMP.getHeading()), DriveConstants.SPECIAL_CONSTRAINTS)
                            .waitFor(1.3)
                            .build());
            trajectories[1][i].add(
                    new TrajectoryBuilder(AutoPaths.CRATER_DUMP, DriveConstants.BASE_CONSTRAINTS)
                            .lineTo(CRATER_TAKE, new GoodLinearInterpolator(AutoPaths.CRATER_DUMP.getHeading(), AutoPaths.CRATER_DUMP.getHeading() - 10 * Math.PI / 180))
                            .build());
            trajectories[1][i].add(
                    new TrajectoryBuilder(trajectories[1][i].get(4).end(), DriveConstants.BASE_CONSTRAINTS)
                            .reverse()
                            .splineTo(CRATER_DUMP)
                            .build());
            trajectories[1][i].add(
                    new TrajectoryBuilder(AutoPaths.CRATER_DUMP, DriveConstants.BASE_CONSTRAINTS)
                            .lineTo(CRATER_TAKE, new GoodLinearInterpolator(AutoPaths.CRATER_DUMP.getHeading(), AutoPaths.CRATER_DUMP.getHeading() + 10 * Math.PI / 180))
                            .build());
            trajectories[1][i].add(
                    new TrajectoryBuilder(trajectories[1][i].get(6).end(), DriveConstants.BASE_CONSTRAINTS)
                            .reverse()
                            .splineTo(CRATER_DUMP)
                            .build());
        }
        //</editor-fold>
    }
}
