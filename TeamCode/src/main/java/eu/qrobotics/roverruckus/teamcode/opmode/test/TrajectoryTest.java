package eu.qrobotics.roverruckus.teamcode.opmode.test;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;

import eu.qrobotics.roverruckus.teamcode.subsystems.Robot;
import eu.qrobotics.roverruckus.teamcode.util.DashboardUtil;
import eu.qrobotics.roverruckus.teamcode.util.ExternalTrajectoryLoader;

@Autonomous
public class TrajectoryTest extends LinearOpMode {

    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        Trajectory trajectory = null;

        try {
            trajectory = ExternalTrajectoryLoader.load("Test");
        } catch (IOException e) {
            Log.wtf("Auto", "SEND HELP");
            e.printStackTrace();
        }

        telemetry.log().add("Ready! Press Play!");
        waitForStart();

        robot.start();

        robot.drive.followTrajectory(trajectory);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }

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
        //telemetry.addData("Update time", (1.0 * robot.lastTime) / 1000000);
        //telemetry.update();
    }
}
