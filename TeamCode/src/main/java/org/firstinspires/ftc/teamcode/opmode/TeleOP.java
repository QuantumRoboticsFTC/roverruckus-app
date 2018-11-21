package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helpers.StickyGamepad;
import org.firstinspires.ftc.teamcode.subsystems.Caruta;
import org.firstinspires.ftc.teamcode.subsystems.Dump;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="TeleOP")
public class TeleOP extends OpMode{
    Robot robot;

    StickyGamepad stickyGamepad2 = null;

    private Boolean isMaturicaStopped = true;


    @Override
    public void init() {
        robot = new Robot(this);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        telemetry.log().add("Hello");
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.drive.setMotorsGamepad(gamepad1);

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
            isMaturicaStopped = !isMaturicaStopped;

            if (isMaturicaStopped) {
                robot.caruta.maturicaMode = Caruta.MaturicaMode.IDLE;
            }
        } else if (gamepad2.right_stick_x < 0) {
            robot.caruta.maturicaMode = Caruta.MaturicaMode.IN;
        } else if (gamepad2.right_stick_x > 0) {
            robot.caruta.maturicaMode = Caruta.MaturicaMode.OUT;
        }

//        robot.caruta.power = gamepad2.right_stick_y;

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

        telemetry.addData("power[0]", robot.drive.powers[0]);
        telemetry.addData("power[1]", robot.drive.powers[1]);
        telemetry.addData("power[2]", robot.drive.powers[2]);
        telemetry.addData("power[3]", robot.drive.powers[3]);
        telemetry.update();

        stickyGamepad2.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
