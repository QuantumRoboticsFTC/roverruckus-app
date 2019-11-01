package eu.qrobotics.roverruckus.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.roverruckus.teamcode.subsystems.Intake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Robot;
import eu.qrobotics.roverruckus.teamcode.util.StickyGamepad;

@TeleOp
public class JustDrive extends OpMode {
    enum DriveMode {
        NORMAL,
        SLOW,
        SUPER_SLOW
    }

    private StickyGamepad stickyGamepad1 = null;
    private DriveMode driveMode;
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, false);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        driveMode = DriveMode.SLOW;
        telemetry.log().add("Ready! Press Play!");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Runtime", getRuntime());
        telemetry.update();
    }

    @Override
    public void start() {
        robot.intake.carutaMode = Intake.CarutaMode.START;
        robot.intake.doorMode = Intake.DoorMode.OPEN;
        robot.outtake.liftDisable = true;
        robot.start();
        telemetry.clear();
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (driveMode) {
            case NORMAL:
                robot.drive.setMotorsGamepad(gamepad1, 1);
                break;
            case SLOW:
                robot.drive.setMotorsGamepad(gamepad1, 0.5);
                break;
            case SUPER_SLOW:
                robot.drive.setMotorsGamepad(gamepad1, 0.25);
                break;
        }

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

        stickyGamepad1.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

}
