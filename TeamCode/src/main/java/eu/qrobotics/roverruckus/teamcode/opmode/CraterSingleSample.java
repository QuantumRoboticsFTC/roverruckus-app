package eu.qrobotics.roverruckus.teamcode.opmode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.roverruckus.teamcode.subsystems.Climb;
import eu.qrobotics.roverruckus.teamcode.subsystems.Intake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Outtake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Robot;
import eu.qrobotics.roverruckus.teamcode.util.DashboardUtil;
import eu.qrobotics.roverruckus.teamcode.vision.MasterVision;
import eu.qrobotics.roverruckus.teamcode.vision.SampleRandomizedPositions;

@Autonomous(group = "Crater")
@Config
public class CraterSingleSample extends LinearOpMode {

    private Robot robot = null;
    public static boolean USE_CAMERA = true;
    public static int WHAT_TRAJECTORY = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        AutoPaths.init();
        robot = new Robot(this, true);
        MasterVision vision = null;
        SampleRandomizedPositions goldPosition = SampleRandomizedPositions.LEFT;

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

        int location = 1;
        switch (goldPosition) {
            case CENTER:
                location = 2;
                break;
            case RIGHT:
                location = 3;
                break;
        }

        Trajectory a = AutoPaths.trajectories[1][location].get(0);
        Trajectory b = AutoPaths.trajectories[1][location].get(1);
        Trajectory c = AutoPaths.trajectories[1][location].get(2);
        Trajectory d = AutoPaths.trajectories[1][location].get(3);
        Trajectory e = AutoPaths.trajectories[1][location].get(4);
        Trajectory f = AutoPaths.trajectories[1][location].get(5);
        Trajectory g = AutoPaths.trajectories[1][location].get(6);

        robot.start();
        robot.intake.goToPositionExtend(0, 0.2);
        robot.climb.setAutonomous();
        robot.climb.setHeight(Climb.MAX_HEIGHT);
        robot.sleep(3.5);
        robot.intake.carutaMode = Intake.CarutaMode.TRANSFER;
        robot.climb.setAutonomous();
        robot.drive.setPoseEstimate(AutoPaths.START_CRATER);

        robot.drive.followTrajectory(a);
        robot.intake.goToPositionExtend(700, 0.115);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }

        robot.climb.setAutonomous();
        robot.climb.setHeight(4);
        while (!isStopRequested() && !robot.intake.isExtendAtTarget()) {
            telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
            telemetry.update();
        }

        robot.intake.carutaMode = Intake.CarutaMode.FLY;
        robot.sleep(0.2);
        robot.intake.maturicaMode = Intake.MaturicaMode.OUT;
        robot.sleep(0.3);

        robot.intake.maturicaMode = Intake.MaturicaMode.IDLE;
        robot.intake.goToPositionExtend(-700, 0.85);
        robot.drive.followTrajectory(b);
        while (!isStopRequested() && (robot.drive.isFollowingTrajectory() || !robot.intake.isExtendAtTarget())) {
            updateDashboard();
        }
        robot.climb.setAutonomous();
        robot.intake.toggleDisable();
        robot.intake.maturicaMode = Intake.MaturicaMode.IN;
        robot.sleep(0.2);

        if (goldPosition != SampleRandomizedPositions.RIGHT) {
            robot.intake.goToPositionExtend(300, 0.6);
            robot.sleep(0.2);
            while (!isStopRequested() && !robot.intake.isExtendAtTarget()) {
                telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
                telemetry.update();
            }
            robot.intake.toggleDisable();
            robot.intake.goToPositionExtend(-300, 0.6);
            robot.sleep(0.2);
            while (!isStopRequested() && !robot.intake.isExtendAtTarget()) {
                telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
                telemetry.update();
            }
        } else {
            robot.intake.goToPositionExtend(525, 0.6);
            robot.sleep(0.2);
            while (!isStopRequested() && !robot.intake.isExtendAtTarget()) {
                telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
                telemetry.update();
            }
            robot.intake.toggleDisable();
            robot.intake.goToPositionExtend(-525, 0.6);
            robot.sleep(0.2);
            while (!isStopRequested() && !robot.intake.isExtendAtTarget()) {
                telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
                telemetry.update();
            }
        }

        robot.intake.carutaMode = Intake.CarutaMode.TRANSFER;
        robot.sleep(0.05);
        robot.intake.maturicaMode = Intake.MaturicaMode.IDLE;

        robot.drive.followTrajectory(c);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }

        dump(Outtake.ScorpionMode.UP, false);
        robot.intake.goToPositionExtend(650, 0.65);
        robot.outtake.scorpionMode = Outtake.ScorpionMode.DOWN;
        robot.outtake.doorMode = Outtake.DoorMode.OPEN;
        robot.outtake.sorterMode = Outtake.SorterMode.OUT;
        robot.sleep(0.4);
        robot.outtake.setLiftPower(-0.2);
        robot.sleep(0.6);
        robot.outtake.setLiftPower(0);

        while (!isStopRequested() && !robot.intake.isExtendAtTarget()) {
            telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
            telemetry.update();
        }

        robot.intake.toggleDisable();
        robot.sleep(0.1);
        robot.drive.followTrajectory(d);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }
        robot.intake.goToPositionExtend(-650, 0.65);
        robot.sleep(0.15);
        robot.intake.toggleDisable();
        robot.sleep(0.05);
        robot.intake.carutaMode = Intake.CarutaMode.TRANSFER;
        robot.sleep(0.05);
        robot.intake.maturicaMode = Intake.MaturicaMode.IDLE;

        robot.drive.followTrajectory(e);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }

        dump(Outtake.ScorpionMode.UP, true);
        robot.intake.goToPositionExtend(650, 0.65);
        robot.outtake.scorpionMode = Outtake.ScorpionMode.DOWN;
        robot.outtake.doorMode = Outtake.DoorMode.OPEN;
        robot.outtake.sorterMode = Outtake.SorterMode.OUT;
        robot.sleep(0.4);
        robot.outtake.setLiftPower(-0.2);
        robot.sleep(0.6);
        robot.outtake.setLiftPower(0);

        while (!isStopRequested() && !robot.intake.isExtendAtTarget()) {
            telemetry.addData("Extend Encoder", robot.intake.getExtendEncoder());
            telemetry.update();
        }

        robot.intake.toggleDisable();
        robot.sleep(0.1);
        robot.drive.followTrajectory(f);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }

        robot.intake.goToPositionExtend(-650, 0.65);
        robot.sleep(0.15);
        robot.intake.toggleDisable();
        robot.sleep(0.05);
        robot.intake.carutaMode = Intake.CarutaMode.TRANSFER;
        robot.sleep(0.05);
        robot.intake.maturicaMode = Intake.MaturicaMode.IDLE;

        robot.drive.followTrajectory(g);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {
            updateDashboard();
        }

        dump(Outtake.ScorpionMode.UP, true);
        robot.intake.goToPositionExtend(750, 0.8);
        robot.outtake.scorpionMode = Outtake.ScorpionMode.DOWN;
        robot.outtake.doorMode = Outtake.DoorMode.OPEN;
        robot.outtake.sorterMode = Outtake.SorterMode.OUT;
        robot.sleep(0.4);
        robot.outtake.setLiftPower(-0.2);
        robot.sleep(0.6);
        robot.outtake.setLiftPower(0);

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

    private void dump(Outtake.ScorpionMode scorpionMode, boolean engageSorter) {
        robot.intake.carutaMode = Intake.CarutaMode.FLY;
        robot.intake.doorMode = Intake.DoorMode.OPEN;
        ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (myTimer.milliseconds() < 1650 && opModeIsActive()) {
            if (80 <= myTimer.milliseconds() && myTimer.milliseconds() < 600 && !robot.outtake.isLiftUp())
                robot.outtake.setLiftPower(1);
            else
                robot.outtake.setLiftPower(0);
            if (120 <= myTimer.milliseconds() && myTimer.milliseconds() <= 200) {
                if (engageSorter)
                    robot.outtake.sorterMode = Outtake.SorterMode.IN;
                robot.outtake.scorpionMode = Outtake.ScorpionMode.MIDDLE;
                robot.outtake.doorMode = Outtake.DoorMode.CLOSE;
            }
            if (850 < myTimer.milliseconds()) {
                robot.outtake.doorMode = Outtake.DoorMode.STRAIGHT;
                robot.outtake.scorpionMode = scorpionMode;
            }
            this.sleep(10);
        }
    }

    private static String formatResults(MovingStatistics statistics) {
        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
                statistics.getMean() * 1000,
                statistics.getStandardDeviation() * 1000,
                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
    }
}
