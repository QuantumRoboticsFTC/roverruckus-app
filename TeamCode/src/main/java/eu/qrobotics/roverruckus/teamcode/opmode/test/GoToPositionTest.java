package eu.qrobotics.roverruckus.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class GoToPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor extendMotor = hardwareMap.get(DcMotor.class, "maturicaExtendMotor");
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendMotor.setDirection(DcMotor.Direction.REVERSE);

        Servo carutaStanga = hardwareMap.get(Servo.class, "carutaLeft");
        Servo carutaDreapta = hardwareMap.get(Servo.class, "carutaRight");
        carutaStanga.setPosition(0.1);
        carutaDreapta.setPosition(0.9);

        waitForStart();

        extendMotor.setTargetPosition(320);
        extendMotor.setPower(1);

        while (opModeIsActive()) {
            idle();
        }
    }
}
