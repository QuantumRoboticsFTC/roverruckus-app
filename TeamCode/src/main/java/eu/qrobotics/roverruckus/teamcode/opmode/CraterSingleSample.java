package eu.qrobotics.roverruckus.teamcode.opmode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.roverruckus.teamcode.navigation.GoodLinearInterpolator;
import eu.qrobotics.roverruckus.teamcode.subsystems.Climb;
import eu.qrobotics.roverruckus.teamcode.subsystems.DriveConstants;
import eu.qrobotics.roverruckus.teamcode.subsystems.Intake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Outtake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Robot;
import eu.qrobotics.roverruckus.teamcode.util.DashboardUtil;
import eu.qrobotics.roverruckus.teamcode.vision.MasterVision;
import eu.qrobotics.roverruckus.teamcode.vision.SampleRandomizedPositions;

@Autonomous
@Config
public class CraterSingleSample extends LinearOpMode {

    private Robot robot = null;
    public static boolean USE_CAMERA = true;
    public static int WHAT_TRAJECTORY = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        MasterVision vision = null;
        SampleRandomizedPositions goldPosition = SampleRandomizedPositions.LEFT;
        DriveConstraints dc = new DriveConstraints(20, 20, Math.PI/2, Math.PI/2);

        robot.drive.toggleAutonomous();
        robot.intake.resetExtend();

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

        if (USE_CAMERA) {
            assert vision != null;
            vision.disable();
            goldPosition = vision.getTfLite().getLastKnownSampleOrder();
        } else
            switch (WHAT_TRAJECTORY) {
                case 1:
                    goldPosition = SampleRandomizedPositions.LEFT;
                    break;
                case 2:
                    goldPosition = SampleRandomizedPositions.CENTER;
                    break;
                case 3:
                    goldPosition = SampleRandomizedPositions.RIGHT;
                    break;
            }

        double secondAngle = AutoPaths.LEFT_SECOND_CRATER;
        switch (goldPosition) {
            case CENTER:
                secondAngle = AutoPaths.CENTER_SECOND_CRATER;
                break;
            case RIGHT:
                secondAngle = AutoPaths.RIGHT_SECOND_CRATER;
                break;
        }

        Trajectory a = new TrajectoryBuilder(AutoPaths.START_CRATER, DriveConstants.BASE_CONSTRAINTS)
                .beginComposite()
                .splineTo(AutoPaths.CRATER_SINGLE_INTERMEDIARY, new GoodLinearInterpolator(AutoPaths.START_CRATER.getHeading(), AutoPaths.CRATER_SINGLE_INTERMEDIARY.getHeading()))
                .splineTo(AutoPaths.CRATER_SINGLE, new GoodLinearInterpolator(AutoPaths.CRATER_SINGLE_INTERMEDIARY.getHeading(), AutoPaths.CRATER_SINGLE.getHeading()))
                .closeComposite()
                .build();
        Trajectory d = new TrajectoryBuilder(a.end(), DriveConstants.BASE_CONSTRAINTS)
                .beginComposite()
                .reverse()
                .splineTo(AutoPaths.CRATER_SINGLE_INTERMEDIARY)
                .splineTo(AutoPaths.CRATER_COLLECT.plus(new Pose2d(0, 0, secondAngle * Math.PI / 180)))
                .closeComposite()
                .build();
        Trajectory e = new TrajectoryBuilder(d.end(), DriveConstants.BASE_CONSTRAINTS)
                .reverse()
                .splineTo(AutoPaths.CRATER_DUMP, new GoodLinearInterpolator(d.end().getHeading(), AutoPaths.CRATER_DUMP.getHeading()), dc)
                .waitFor(1)
                .build();
        Trajectory f = new TrajectoryBuilder(e.end(), DriveConstants.BASE_CONSTRAINTS)
                .forward(10)
                .build();


        robot.start();
        robot.intake.goToPositionExtend(0, 0.2);
        robot.climb.setAutonomous();
        robot.climb.setHeight(Climb.MAX_HEIGHT);
        robot.sleep(3.5);
        robot.intake.carutaMode = Intake.CarutaMode.TRANSFER;
        robot.climb.setAutonomous();
        robot.drive.setPoseEstimate(AutoPaths.START_CRATER);

        robot.drive.followTrajectory(a);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }

        robot.intake.goToPositionExtend(700, 0.75);
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

        robot.intake.goToPositionExtend(-700, 0.5);
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
        robot.climb.setAutonomous();
        robot.climb.setHeight(4);

        if (goldPosition != SampleRandomizedPositions.RIGHT) {
            robot.intake.goToPositionExtend(350, 0.4);
            robot.sleep(0.2);
            while (!isStopRequested() && !robot.intake.isExtendAtTarget()) { // poate sa dispara o secunda ~
                telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
                telemetry.update();
            }
            robot.intake.goToPositionExtend(-350, 0.4);
            robot.sleep(0.2);
            while (!isStopRequested() && !robot.intake.isExtendAtTarget()) { // poate sa dispara o secunda ~
                telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
                telemetry.update();
            }
        } else {
            robot.intake.goToPositionExtend(550, 0.4);
            robot.sleep(0.2);
            while (!isStopRequested() && !robot.intake.isExtendAtTarget()) { // poate sa dispara o secunda ~
                telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
                telemetry.update();
            }
            robot.intake.goToPositionExtend(-550, 0.4);
            robot.sleep(0.2);
            while (!isStopRequested() && !robot.intake.isExtendAtTarget()) { // poate sa dispara o secunda ~
                telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
                telemetry.update();
            }
        }

        robot.intake.toggleDisable();
        robot.sleep(0.2);
        robot.intake.carutaMode = Intake.CarutaMode.TRANSFER;
        robot.sleep(0.7);
        robot.climb.setAutonomous();

        robot.outtake.doorMode = Outtake.DoorMode.TRANSFER;
        robot.intake.carutaMode = Intake.CarutaMode.FLY;
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
        robot.outtake.setLiftPower(-0.2);
        robot.sleep(0.6);
        robot.outtake.setLiftPower(0);

        robot.intake.goToPositionExtend(675, 0.8);
        robot.sleep(0.2);
        while (!isStopRequested() && !robot.intake.isExtendAtTarget()) { // poate sa dispara o secunda ~
            telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
            telemetry.update();
        }

        robot.intake.toggleDisable();
        robot.sleep(0.1);
        robot.drive.followTrajectory(f);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }

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
