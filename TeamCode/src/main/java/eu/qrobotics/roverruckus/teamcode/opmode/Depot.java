package eu.qrobotics.roverruckus.teamcode.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
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
import eu.qrobotics.roverruckus.teamcode.util.DashboardUtil;
import eu.qrobotics.roverruckus.teamcode.util.ExternalTrajectoryLoader;
import eu.qrobotics.roverruckus.teamcode.vision.MasterVision;
import eu.qrobotics.roverruckus.teamcode.vision.SampleRandomizedPositions;

@Config
@Autonomous(name = "Depot")
public class Depot extends LinearOpMode {

    // 1 - left, 2 - mid, 3 - right
    public static boolean USE_CAMERA = false;
    public static int WHAT_TRAJECTORY = 2;
    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        Trajectory left = null;
        Trajectory leftBack = null;
        Trajectory mid = null;
        Trajectory midBack = null;
        Trajectory right = null;
        Trajectory rightBack = null;
        Trajectory first;
        Trajectory second;
        MasterVision vision = null;
        SampleRandomizedPositions goldPosition;

        try {
            left = ExternalTrajectoryLoader.load("DepotLeft");
            leftBack = ExternalTrajectoryLoader.load("DepotLeftBack");
            mid = ExternalTrajectoryLoader.load("DepotMid");
            midBack = ExternalTrajectoryLoader.load("DepotMidBack");
            right = ExternalTrajectoryLoader.load("DepotRight");
            rightBack = ExternalTrajectoryLoader.load("DepotRightBack");
        } catch (IOException e) {
            Log.wtf("Auto", "SEND HELP");
            e.printStackTrace();
        }

        if (USE_CAMERA) {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = Robot.VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
            vision = new MasterVision(parameters, hardwareMap, false, MasterVision.TFLiteAlgorithm.INFER_RIGHT);
            vision.init();
            vision.enable();
        }

        telemetry.log().add("Ready! Press Play!");

        waitForStart();
        if (isStopRequested())
            return;
        robot.start();

        if (USE_CAMERA) {
            assert vision != null;
            vision.disable();
            goldPosition = vision.getTfLite().getLastKnownSampleOrder();
            telemetry.log().clear();
            telemetry.log().add(goldPosition.name());

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
        } else {
            switch (WHAT_TRAJECTORY) {
                case 1:
                    first = left;
                    second = leftBack;
                    break;
                case 3:
                    first = right;
                    second = rightBack;
                    break;
                default:
                    first = mid;
                    second = midBack;
                    break;
            }
        }

        robot.drive.followTrajectory(first);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory())
            updateDashboard();

        if (isStopRequested()) {
            robot.stop();
            return;
        }

        robot.caruta.carutaMode = Caruta.CarutaMode.DOWN;
        robot.sleep(1);

        robot.caruta.carutaMode = Caruta.CarutaMode.UP;
        robot.sleep(1);

        robot.drive.followTrajectory(second);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory())
            updateDashboard();

        robot.stop();
    }

    private void updateDashboard() {
        robot.drive.updatePoseEstimate();
        robot.drive.updateFollower();
        Pose2d currentPose = robot.drive.getPoseEstimate();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());
        packet.put("update time", (1.0 * robot.lastTime) / 1000000);

        fieldOverlay.setStrokeWidth(4);
        fieldOverlay.setStroke("green");
        DashboardUtil.drawSampledTrajectory(fieldOverlay, robot.drive.lastTrajectory);

        fieldOverlay.setFill("blue");
        fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

        robot.dashboard.sendTelemetryPacket(packet);
    }
}
