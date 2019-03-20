package eu.qrobotics.roverruckus.teamcode.opmode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.roverruckus.teamcode.navigation.GoodLinearInterpolator;
import eu.qrobotics.roverruckus.teamcode.subsystems.DriveConstants;
import eu.qrobotics.roverruckus.teamcode.subsystems.Intake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Outtake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Robot;
import eu.qrobotics.roverruckus.teamcode.util.DashboardUtil;

@Autonomous
public class CraterDoubleSample extends LinearOpMode {

    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        DriveConstraints dc = new DriveConstraints(20, 20, Math.PI/2, Math.PI/2);

        Trajectory a = new TrajectoryBuilder(AutoPaths.START_CRATER, DriveConstants.BASE_CONSTRAINTS)
                .splineTo(AutoPaths.CRATER_SECOND)
                .build();
        Trajectory b = new TrajectoryBuilder(AutoPaths.CRATER_SECOND, DriveConstants.BASE_CONSTRAINTS)
                .turn(AutoPaths.LEFT_SECOND_DEPOT * Math.PI / 180)
                .build();
        Trajectory c = new TrajectoryBuilder(b.end(), DriveConstants.BASE_CONSTRAINTS)
                .forward(9)
                .build();
        Trajectory d = new TrajectoryBuilder(c.end(), DriveConstants.BASE_CONSTRAINTS)
                .reverse()
                .splineTo(AutoPaths.CRATER_COLLECT.plus(new Pose2d(0, 0, AutoPaths.LEFT_SECOND_CRATER * Math.PI / 180)))
                .build();
        Trajectory e = new TrajectoryBuilder(d.end(), DriveConstants.BASE_CONSTRAINTS)
                .reverse()
                .splineTo(AutoPaths.CRATER_DUMP, new GoodLinearInterpolator(d.end().getHeading(), AutoPaths.CRATER_DUMP.getHeading()), dc)
                .waitFor(1)
                .build();

        robot.drive.toggleAutonomous();
        robot.drive.setPoseEstimate(AutoPaths.START_CRATER);
        telemetry.log().add("Ready! Press Play!");
        waitForStart();

        robot.start();
        robot.intake.goToPositionExtend(0, 0.2);

        robot.drive.followTrajectory(a);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }

        robot.intake.goToPositionExtend(800, 0.75);
        robot.sleep(0.2);
        while(!isStopRequested() && !robot.intake.isExtendAtTarget()) { // poate sa dispara o secunda ~
            telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
            telemetry.update();
        }

        robot.intake.carutaMode = Intake.CarutaMode.FLY;
        robot.intake.maturicaMode = Intake.MaturicaMode.FAST_OUT;
        robot.sleep(0.4);

        robot.intake.maturicaMode = Intake.MaturicaMode.IDLE;
        robot.sleep(0.3);

        robot.drive.followTrajectory(b);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }
        robot.intake.toggleDisable();
        robot.intake.maturicaMode = Intake.MaturicaMode.IN;
        robot.sleep(0.3);

        robot.drive.followTrajectory(c);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }

        robot.intake.carutaMode = Intake.CarutaMode.FLY;
        robot.sleep(0.2);
        robot.intake.goToPositionExtend(-800, 0.75);
        robot.sleep(0.2);
        while(!isStopRequested() && !robot.intake.isExtendAtTarget()) { // poate sa dispara o secunda ~
            telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
            telemetry.update();
        }

        robot.drive.followTrajectory(d);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }
        robot.intake.toggleDisable();
        robot.intake.maturicaMode = Intake.MaturicaMode.IN;
        robot.intake.goToPositionExtend(350, 0.4);
        robot.sleep(0.2);
        while(!isStopRequested() && !robot.intake.isExtendAtTarget()) { // poate sa dispara o secunda ~
            telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
            telemetry.update();
        }
        robot.intake.goToPositionExtend(-350, 0.4);
        robot.sleep(0.2);
        while(!isStopRequested() && !robot.intake.isExtendAtTarget()) { // poate sa dispara o secunda ~
            telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
            telemetry.update();
        }
        robot.intake.carutaMode = Intake.CarutaMode.TRANSFER;
        robot.sleep(0.8);

        robot.outtake.doorMode = Outtake.DoorMode.TRANSFER;
        robot.drive.followTrajectory(e);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }


        robot.outtake.setLiftPower(1);
        while(!isStopRequested() && !robot.outtake.isLiftUp()) {
            updateDashboard();
        }
        robot.outtake.scorpionMode = Outtake.ScorpionMode.MIDDLE;
        robot.outtake.doorMode = Outtake.DoorMode.CLOSE;
        robot.sleep(1);
        robot.outtake.doorMode = Outtake.DoorMode.STRAIGHT;
        robot.outtake.scorpionMode = Outtake.ScorpionMode.UP;
        robot.sleep(0.5);
        robot.outtake.scorpionMode = Outtake.ScorpionMode.DOWN;
        robot.outtake.doorMode = Outtake.DoorMode.OPEN;
        robot.sleep(0.4);
        robot.outtake.setLiftPower(-0.3);
        robot.sleep(0.6);

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
