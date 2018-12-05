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

@Autonomous(name = "Crater")
public class Crater extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        Trajectory left = null;
        Trajectory mid = null;
        Trajectory right = null;
        Trajectory back = null;
        MasterVision vision;
        SampleRandomizedPositions goldPosition;

        try {
            left = AssetsTrajectoryLoader.load("CraterLeft");
            mid = AssetsTrajectoryLoader.load("CraterMid");
            right = AssetsTrajectoryLoader.load("CraterRight");
            back = AssetsTrajectoryLoader.load("CraterBack");
        } catch (IOException e) {
            Log.wtf("Auto", "SEND HELP");
            e.printStackTrace();
        }

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = Robot.VUFORIA_KEY;
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

        Trajectory temp;
        switch (goldPosition) {
            case LEFT:
                temp = left;
                break;
            case CENTER:
                temp = mid;
                break;
            default:
                temp = right;
                break;
        }

        robot.drive.followTrajectory(temp);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            Pose2d currentPose = robot.drive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());

            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, temp);

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            robot.dashboard.sendTelemetryPacket(packet);
        }

        robot.caruta.carutaMode = Caruta.CarutaMode.DOWN;
        robot.sleep(1);
        robot.caruta.carutaMode = Caruta.CarutaMode.UP;
        robot.sleep(1);

        robot.drive.followTrajectory(back);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            Pose2d currentPose = robot.drive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());

            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, back);

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            robot.dashboard.sendTelemetryPacket(packet);
        }

        robot.stop();
    }
}
