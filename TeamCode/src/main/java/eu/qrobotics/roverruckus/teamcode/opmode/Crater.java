package eu.qrobotics.roverruckus.teamcode.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.io.IOException;

import eu.qrobotics.roverruckus.teamcode.subsystems.Climb;
import eu.qrobotics.roverruckus.teamcode.subsystems.Intake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Robot;
import eu.qrobotics.roverruckus.teamcode.util.DashboardUtil;
import eu.qrobotics.roverruckus.teamcode.util.ExternalTrajectoryLoader;
import eu.qrobotics.roverruckus.teamcode.vision.MasterVision;
import eu.qrobotics.roverruckus.teamcode.vision.SampleRandomizedPositions;

@Config
@Autonomous(name = "Crater", group = "Crater")
@Disabled
public class Crater extends LinearOpMode {

    // 1 - left, 2 - mid, 3 - right
    public static boolean USE_CAMERA = true;
    public static int WHAT_TRAJECTORY = 3;
    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        Trajectory left = null;
        Trajectory mid = null;
        Trajectory right = null;
        Trajectory back = null;
        Trajectory temp;
        MasterVision vision = null;
        SampleRandomizedPositions goldPosition;

        robot.drive.toggleAutonomous();
        robot.intake.resetExtend();

        try {
            left = ExternalTrajectoryLoader.load("CraterLeft");
            mid = ExternalTrajectoryLoader.load("CraterMid");
            right = ExternalTrajectoryLoader.load("CraterRight");
            back = ExternalTrajectoryLoader.load("CraterBack");
        } catch (IOException e) {
            Log.wtf("Auto", "SEND HELP");
            e.printStackTrace();
        }

        if (USE_CAMERA) {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = Robot.VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
            vision = new MasterVision(parameters, hardwareMap, false, MasterVision.TFLiteAlgorithm.INFER_NONE);
            vision.init();
            vision.enable();
        }

        telemetry.log().add("Ready! Press Play!");

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }

        telemetry.log().clear();

        if (isStopRequested())
            return;

        robot.start();
        robot.intake.setExtendPower(-0.1);

        if (USE_CAMERA) {
            assert vision != null;
            vision.disable();
            goldPosition = vision.getTfLite().getLastKnownSampleOrder();
            telemetry.log().add(goldPosition.name());
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
        } else {
            switch (WHAT_TRAJECTORY) {
                case 1:
                    temp = left;
                    break;
                case 2:
                    temp = mid;
                    break;
                default:
                    temp = right;
                    break;
            }
        }

        robot.climb.setAutonomous();
        robot.climb.setHeight(Climb.MAX_HEIGHT);
        robot.sleep(3.5);
        robot.climb.setAutonomous();
        robot.drive.setPoseEstimate(new Pose2d(14.3, 14.3, Math.PI / 4));
        /*double startTime = getRuntime();

        while (!isStopRequested() && startTime + 8 > getRuntime())
            updateDashboard();

        if (isStopRequested()) {
            robot.stop();
            return;
        }*/

        robot.drive.followTrajectory(temp);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory())
            updateDashboard();

        if (isStopRequested()) {
            robot.stop();
            return;
        }

        robot.intake.carutaMode = Intake.CarutaMode.FLY;
        robot.sleep(0.2);

        robot.intake.maturicaMode = Intake.MaturicaMode.OUT;
        robot.sleep(0.3);

        robot.intake.carutaMode = Intake.CarutaMode.START;
        robot.intake.maturicaMode = Intake.MaturicaMode.IDLE;
        robot.sleep(0.3);

        robot.drive.followTrajectory(back);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory())
            updateDashboard();

        robot.intake.carutaMode = Intake.CarutaMode.FLY;
        robot.sleep(1);

        robot.drive.toggleAutonomous();
        robot.climb.setAutonomous();
        robot.climb.setHeight(4);
        robot.sleep(0.1);
        robot.intake.setExtendPower(0.5);
        robot.sleep(1);
        robot.intake.setExtendPower(0);
        robot.sleep(2);
        robot.climb.setAutonomous();

        robot.stop();
    }

    private void updateDashboard() {
        Pose2d currentPose = robot.drive.getPoseEstimate();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        fieldOverlay.setStrokeWidth(4);
        fieldOverlay.setStroke("green");
        DashboardUtil.drawSampledTrajectory(fieldOverlay, robot.drive.getTrajectory());

        fieldOverlay.setFill("blue");
        fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

        robot.dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Top 250", formatResults(robot.top250));
        telemetry.addData("Top 100", formatResults(robot.top100));
        telemetry.addData("Top 10", formatResults(robot.top10));
        telemetry.update();
    }

    private static String formatResults(MovingStatistics statistics) {
        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
                statistics.getMean() * 1000,
                statistics.getStandardDeviation() * 1000,
                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
    }
}
