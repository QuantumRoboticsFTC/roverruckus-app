package eu.qrobotics.roverruckus.teamcode.opmode.test;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.roverruckus.teamcode.opmode.AutoPaths;
import eu.qrobotics.roverruckus.teamcode.subsystems.DriveConstants;
import eu.qrobotics.roverruckus.teamcode.subsystems.Robot;
import eu.qrobotics.roverruckus.teamcode.util.DashboardUtil;

@Autonomous(group = "Test")
public class TrajectoryTest extends LinearOpMode {

    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);

        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(AutoPaths.START_CRATER, DriveConstants.BASE_CONSTRAINTS);
        Trajectory trajectory = trajectoryBuilder
                .splineTo(new Pose2d(0, 50, Math.PI)/*, new GoodLinearInterpolator(AutoPaths.START_CRATER.getHeading(), Math.PI)*/)
                .build();

        robot.drive.toggleAutonomous();
        robot.drive.setPoseEstimate(AutoPaths.START_CRATER);
        telemetry.log().add("Ready! Press Play!");
        waitForStart();

        robot.start();

        robot.drive.followTrajectory(trajectory);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }
        robot.sleep(2);

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
