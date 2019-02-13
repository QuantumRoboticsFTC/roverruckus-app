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
    private boolean up = false;
    private boolean up_down = false;
    private boolean climb = true;

    @Override
    public void init() {
        robot = new Robot(this);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        driveMode = DriveMode.NORMAL;
        telemetry.log().add("Ready! Press Play!");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Runtime", getRuntime());
        telemetry.update();
        super.init_loop();
    }

    @Override
    public void start() {
        robot.intake.carutaMode = Intake.CarutaMode.FLY;
        robot.start();
        telemetry.log().clear();
    }

    @Override
    public void loop() {
        //MARK: drive power update
        //PRECHECK: ok
        switch (driveMode) {
            case NORMAL:
                robot.drive.setMotorsGamepad(gamepad1, 0.9);
                break;
            case SLOW:
                robot.drive.setMotorsGamepad(gamepad1, 0.5);
                break;
            case SUPER_SLOW:
                robot.drive.setMotorsGamepad(gamepad1, 0.25);
                break;
        }

        //MARK: toggle between climb and intake on gamepad1
        if (stickyGamepad1.dpad_up)
            climb = true;
        if (stickyGamepad1.dpad_down)
            climb = false;

        //MARK: drive speed mode
        //PRECHECK: ok
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

        //MARK: climb and intake control
        //PRECHECK: ok
        if (climb) {
            if (gamepad1.right_trigger > 0.01)
                robot.climb.globalPower = gamepad1.right_trigger;
            else if (gamepad1.left_trigger > 0.01)
                robot.climb.globalPower = -gamepad1.left_trigger;
            else
                robot.climb.globalPower = 0;

            robot.intake.setExtendPower(-gamepad2.right_stick_y);
        } else {
            if (gamepad1.right_trigger > 0.01)
                robot.intake.setExtendPower(gamepad1.right_trigger);
            else if (gamepad1.left_trigger > 0.01)
                robot.intake.setExtendPower(-gamepad1.left_trigger);
            else
                robot.intake.setExtendPower(0);

            robot.climb.globalPower = -gamepad2.right_stick_y;
        }

        //MARK: intake intake
        //PRECHECK: ok
        if (stickyGamepad2.a) {
            if (robot.intake.carutaMode != Intake.CarutaMode.FLY)
                robot.intake.carutaMode = Intake.CarutaMode.FLY;
            else
                robot.intake.carutaMode = Intake.CarutaMode.TRANSFER;
        }

        //MARK: disable intake
        //PRECHECK: ok
        if (stickyGamepad2.b)
            robot.intake.toggleDisable();

        if (stickyGamepad2.y)
            robot.intake.carutaMode = Intake.CarutaMode.START;

        //MARK: intake maturice
        //PRECHECK: ok
        if (gamepad2.left_stick_y < -0.2) {
            robot.intake.maturicaMode = Intake.MaturicaMode.IN;
        } else if (gamepad2.left_stick_y > 0.2) {
            robot.intake.maturicaMode = Intake.MaturicaMode.OUT;
        } else if (((stickyGamepad2.right_stick_button || stickyGamepad2.right_bumper)
                && robot.intake.maturicaMode == Intake.MaturicaMode.IN)
                || robot.intake.maturicaMode == Intake.MaturicaMode.OUT)
            robot.intake.maturicaMode = Intake.MaturicaMode.IDLE;

        //MARK: outtake lift
        //PRECHECK: ok
        if (gamepad2.right_trigger > 0)
            robot.outtake.setLiftPower(gamepad2.right_trigger);
        else if (gamepad2.left_trigger > 0)
            robot.outtake.setLiftPower(-gamepad2.left_trigger * 0.25);
        else
            robot.outtake.setLiftPower(0);

        //MARK: scorpion door
        if (stickyGamepad2.x) {
            if (robot.outtake.doorMode == Outtake.DoorMode.CLOSE) {
                if (up)
                    robot.outtake.doorMode = Outtake.DoorMode.STRAIGHT;
                else
                    robot.outtake.doorMode = Outtake.DoorMode.OPEN;
            } else
                robot.outtake.doorMode = Outtake.DoorMode.CLOSE;
        }

        //MARK: scorpion down
        if (stickyGamepad2.left_bumper) {
            robot.outtake.sorterMode = Outtake.SorterMode.OUT;
            robot.outtake.scorpionMode = Outtake.ScorpionMode.DOWN;
            robot.outtake.doorMode = Outtake.DoorMode.OPEN;
            up = false;
        }

        //MARK: scorpion automatic flip
        if (!up_down && robot.getRevBulkDataHub2().getDigitalInputState(robot.outtake.liftSwitch)) {
            up = true;
            up_down = true;
            robot.outtake.sorterMode = Outtake.SorterMode.IN;
            robot.outtake.scorpionMode = Outtake.ScorpionMode.UP;
        } else if (!up && !robot.getRevBulkDataHub2().getDigitalInputState(robot.outtake.liftSwitch)) {
//            up = false;
            up_down = false;
//            robot.outtake.sorterMode = Outtake.SorterMode.OUT;
//            robot.outtake.scorpionMode = Outtake.ScorpionMode.DOWN;
//            robot.outtake.doorMode = Outtake.DoorMode.OPEN;
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
