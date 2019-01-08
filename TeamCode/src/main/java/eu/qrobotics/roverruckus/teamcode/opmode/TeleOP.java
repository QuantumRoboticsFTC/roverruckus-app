package eu.qrobotics.roverruckus.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import eu.qrobotics.roverruckus.teamcode.subsystems.Intake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Outtake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Robot;
import eu.qrobotics.roverruckus.teamcode.util.StickyGamepad;

@TeleOp(name = "TeleOP")
public class TeleOP extends OpMode {
    enum DriveMode {
        NORMAL,
        SLOW,
        SUPER_SLOW
    }

    private Robot robot = null;
    private StickyGamepad stickyGamepad1 = null;
    private StickyGamepad stickyGamepad2 = null;
    private DriveMode driveMode;

    @Override
    public void init() {
        robot = new Robot(this);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        driveMode = DriveMode.NORMAL;
        telemetry.log().add("Ready! Press Play!");
    }

    @Override
    public void start() {
        robot.start();
        telemetry.log().clear();
    }

    @Override
    public void loop() {
        //MARK: drive power update
        switch (driveMode) {
            case NORMAL:
                robot.drive.setMotorsGamepad(gamepad1, 0.85);
                break;
            case SLOW:
                robot.drive.setMotorsGamepad(gamepad1, 0.45);
                break;
            case SUPER_SLOW:
                robot.drive.setMotorsGamepad(gamepad1, 0.25);
                break;
        }

        //MARK: drive speed mode
        if (stickyGamepad1.a) {
            if (driveMode != DriveMode.SLOW)
                driveMode = DriveMode.SLOW;
            else
                driveMode = DriveMode.NORMAL;
        } else if (stickyGamepad1.b) {
            if (driveMode != DriveMode.SUPER_SLOW)
                driveMode = DriveMode.SUPER_SLOW;
            else
                driveMode = DriveMode.NORMAL;
        }

        //MARK: intake extend
        if (gamepad2.dpad_up) {
            robot.caruta.extendMode = Intake.ExtendMode.FORWARD;
        } else if (gamepad2.dpad_down) {
            robot.caruta.extendMode = Intake.ExtendMode.BACK;
        } else {
            robot.caruta.extendMode = Intake.ExtendMode.IDLE;
        }

        //MARK: intake maturice
        if (stickyGamepad2.right_stick_button) {
            robot.caruta.maturicaMode = Intake.MaturicaMode.IDLE;
        } else if (gamepad2.right_stick_y < -0.2) {
            robot.caruta.maturicaMode = Intake.MaturicaMode.IN;
        } else if (gamepad2.right_stick_y > 0.2) {
            robot.caruta.maturicaMode = Intake.MaturicaMode.OUT;
        }

        //MARK: intake caruta
        if (stickyGamepad2.a) {
            if (robot.caruta.carutaMode == Intake.CarutaMode.DISABLE || robot.caruta.carutaMode == Intake.CarutaMode.DOWN) {
                robot.caruta.carutaMode = Intake.CarutaMode.UP;
            } else {
                robot.caruta.carutaMode = Intake.CarutaMode.DOWN;
            }
        }

        //MARK: disable caruta
        if (stickyGamepad2.b) {
            robot.caruta.toggleDisable();
        }

        //MARK: outtake climb
        if (gamepad2.right_trigger > 0) {
            robot.outtake.liftMode = Outtake.LiftMode.UP;
        } else if (gamepad2.left_trigger > 0) {
            robot.outtake.liftMode = Outtake.LiftMode.DOWN;
        } else {
            robot.outtake.liftMode = Outtake.LiftMode.IDLE;
        }

        if (stickyGamepad2.x) {
            if (robot.outtake.dumpMode == Outtake.DumpMode.UP) {
                robot.outtake.dumpMode = Outtake.DumpMode.DOWN;
            } else {
                robot.outtake.dumpMode = Outtake.DumpMode.UP;
            }
        }

        //MARK: Telemetry
        telemetry.addData("Drive Mode", driveMode);
        telemetry.addData("Top 250", formatResults(robot.top250));
        telemetry.addData("Top 100", formatResults(robot.top100));
        telemetry.addData("Top 10", formatResults(robot.top10));
        telemetry.update();

        //MARK: Update sticky gamepads
        stickyGamepad1.update();
        stickyGamepad2.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private static String formatResults(MovingStatistics statistics) {
        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
                statistics.getMean() * 1000,
                statistics.getStandardDeviation() * 1000,
                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
    }
}
