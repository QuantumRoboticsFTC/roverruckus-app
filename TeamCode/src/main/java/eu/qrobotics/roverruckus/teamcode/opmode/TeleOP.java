package eu.qrobotics.roverruckus.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.roverruckus.teamcode.subsystems.Caruta;
import eu.qrobotics.roverruckus.teamcode.subsystems.Dump;
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
            robot.caruta.extendMode = Caruta.ExtendMode.FORWARD;
        } else if (gamepad2.dpad_down) {
            robot.caruta.extendMode = Caruta.ExtendMode.BACK;
        } else {
            robot.caruta.extendMode = Caruta.ExtendMode.IDLE;
        }

        //MARK: intake maturice
        if (stickyGamepad2.right_stick_button) {
            robot.caruta.maturicaMode = Caruta.MaturicaMode.IDLE;
        } else if (gamepad2.right_stick_y < -0.2) {
            robot.caruta.maturicaMode = Caruta.MaturicaMode.IN;
        } else if (gamepad2.right_stick_y > 0.2) {
            robot.caruta.maturicaMode = Caruta.MaturicaMode.OUT;
        }

        //MARK: intake caruta
        if (stickyGamepad2.a) {
            if (robot.caruta.carutaMode == Caruta.CarutaMode.DISABLE || robot.caruta.carutaMode == Caruta.CarutaMode.DOWN) {
                robot.caruta.carutaMode = Caruta.CarutaMode.UP;
            } else {
                robot.caruta.carutaMode = Caruta.CarutaMode.DOWN;
            }
        }

        //MARK: disable caruta
        if (stickyGamepad2.b) {
            robot.caruta.toggleDisable();
        }

        //MARK: outtake climb
        if (gamepad2.right_trigger > 0) {
            robot.dump.climbMode = Dump.ClimbMode.UP;
        } else if (gamepad2.left_trigger > 0) {
            robot.dump.climbMode = Dump.ClimbMode.DOWN;
        } else {
            robot.dump.climbMode = Dump.ClimbMode.IDLE;
        }

        if (stickyGamepad2.x) {
            if (robot.dump.dumpMode == Dump.DumpMode.UP) {
                robot.dump.dumpMode = Dump.DumpMode.DOWN;
            } else {
                robot.dump.dumpMode = Dump.DumpMode.UP;
            }
        }

        //MARK: Telemetry
        telemetry.addData("Drive Mode", driveMode);
        telemetry.update();

        //MARK: Update sticky gamepads
        stickyGamepad1.update();
        stickyGamepad2.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
