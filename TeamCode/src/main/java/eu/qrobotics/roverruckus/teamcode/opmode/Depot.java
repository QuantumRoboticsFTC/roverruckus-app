package eu.qrobotics.roverruckus.teamcode.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.IOException;

import eu.qrobotics.roverruckus.teamcode.subsystems.Caruta;
import eu.qrobotics.roverruckus.teamcode.subsystems.Robot;
import eu.qrobotics.roverruckus.teamcode.util.AssetsTrajectoryLoader;
import eu.qrobotics.roverruckus.teamcode.util.DashboardUtil;
import eu.qrobotics.roverruckus.teamcode.vision.MasterVision;
import eu.qrobotics.roverruckus.teamcode.vision.SampleRandomizedPositions;

@Autonomous(name = "Depot")
public class Depot extends LinearOpMode {

    private static final String VUFORIA_KEY = "AZeyMpr/////AAABmbEShUcKiUwjoZ6YAwEmv09dz3UqbaI1dYslOuqZi1df8jYNLKBadOXeFjLRI/cJuDvGJC2VLghjm+sIy2YJf3nDHzg8jMZTTp1QPHPtXIIrEpOra6eimb69W+VCjZwW+lR1HyPVX56wJjgcThGEcdqC9j/rQoSKoKFCY+rZOOxG30rqJuYW3wKO97vlepFai4uTZ67Ipm7T9Hfrc+bUWZd+g2BqmweHvtDlixyab8TKsc8wTduOQBJ5Nxrh8ZmuCw/3SNZgIxOkacjzSJAT3dip2Q/VeKl1CG5SmxYy92GlwWWELm6tkCq9b+cS1GxvetnmsoRWUz0/oj/cuu0eRsrQmrYj3bV2Y9f7qecxgMSE";

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        Trajectory left = null;
        Trajectory leftBack = null;
        Trajectory mid = null;
        Trajectory midBack = null;
        Trajectory right = null;
        Trajectory rightBack = null;
        MasterVision vision;
        SampleRandomizedPositions goldPosition;

        try {
            left = AssetsTrajectoryLoader.load("DepotLeft");
            leftBack = AssetsTrajectoryLoader.load("DepotLeftBack");
            mid = AssetsTrajectoryLoader.load("DepotMid");
            midBack = AssetsTrajectoryLoader.load("DepotMidBack");
            right = AssetsTrajectoryLoader.load("DepotRight");
            rightBack = AssetsTrajectoryLoader.load("DepotRightBack");
        } catch (IOException e) {
            Log.wtf("Auto", "SEND HELP");
            e.printStackTrace();
        }

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vision = new MasterVision(parameters, hardwareMap, false, MasterVision.TFLiteAlgorithm.INFER_RIGHT);
        vision.init();
        vision.enable();

        telemetry.log().add("Ready! Press Play!");

        waitForStart();

        vision.disable();
        goldPosition = vision.getTfLite().getLastKnownSampleOrder();
        telemetry.log().clear();
        telemetry.log().add(goldPosition.name());

        robot.start();

        Trajectory first;
        Trajectory second;
        switch (goldPosition) {
            case LEFT:
                first = left;
                second = leftBack;
                break;
            case RIGHT:
                first = right;
                second = rightBack;
                break;
            default:
                first = mid;
                second = midBack;
                break;
        }

        robot.drive.followTrajectory(first);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            Pose2d currentPose = robot.drive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());

            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, first);

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            robot.dashboard.sendTelemetryPacket(packet);
        }

        robot.caruta.carutaMode = Caruta.CarutaMode.DOWN;
        robot.sleep(1);
        robot.caruta.carutaMode = Caruta.CarutaMode.UP;
        robot.sleep(1);

        robot.drive.followTrajectory(second);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            Pose2d currentPose = robot.drive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());

            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, second);

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            robot.dashboard.sendTelemetryPacket(packet);
        }

        robot.stop();
    }
}
