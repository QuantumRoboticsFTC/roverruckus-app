package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Caruta implements Subsystem {

    public enum ExtendMode {
        FORWARD,
        IDLE,
        BACK
    }

    public enum CarutaMode {
        UP,
        DOWN,
        DISABLE
    }

    public enum MaturicaMode {
        IN,
        IDLE,
        OUT
    }

    public MaturicaMode maturicaMode;
    public CarutaMode carutaMode;
    public ExtendMode extendMode;
    public double power;

    private DcMotorEx extendMotor = null;
    private DcMotorEx maturicaMotor = null;
    private Servo carutaStanga = null;
    private Servo carutaDreapta = null;

    public Caruta(HardwareMap hardwareMap) {
        maturicaMotor = hardwareMap.get(DcMotorEx.class, "maturicaMotor");
        extendMotor = hardwareMap.get(DcMotorEx.class, "maturicaExtendMotor");

        carutaStanga = hardwareMap.get(Servo.class, "carutaLeft");
        carutaDreapta = hardwareMap.get(Servo.class, "carutaRight");

        maturicaMotor.setDirection(DcMotorEx.Direction.REVERSE);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //maturicaMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        maturicaMode = MaturicaMode.IDLE;
        carutaMode = CarutaMode.UP;
        extendMode = ExtendMode.IDLE;
        power = 0;
    }

    public void toggleDisable() {
        if (carutaMode != CarutaMode.DISABLE) {
            carutaMode = CarutaMode.DISABLE;
            carutaStanga.getController().pwmDisable();
        } else {
            carutaMode = CarutaMode.DOWN;
            carutaStanga.getController().pwmEnable();
        }
    }

    @Override
    public void update() {
        switch (maturicaMode) {
            case IN:
                maturicaMotor.setPower(-1);
                break;
            case IDLE:
                maturicaMotor.setPower(0);
                break;
            case OUT:
                maturicaMotor.setPower(1);
                break;
        }
//        maturicaMotor.setPower(power);

        switch (carutaMode) {
            case UP:
                carutaStanga.setPosition(0.275);
                carutaDreapta.setPosition(0.75);
                break;
            case DOWN:
                carutaStanga.setPosition(0.75);
                carutaDreapta.setPosition(0.25);
                break;
            case DISABLE:
                break;
        }

        switch (extendMode) {
            case IDLE:
                extendMotor.setPower(0);
                break;
            case FORWARD:
                extendMotor.setPower(0.25);
                break;
            case BACK:
                extendMotor.setPower(-0.25);
                break;
        }
    }


}
