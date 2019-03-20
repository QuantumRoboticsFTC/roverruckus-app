package eu.qrobotics.roverruckus.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import eu.qrobotics.roverruckus.teamcode.hardware.CachingDcMotorEx;

@Config
public class Intake implements Subsystem {
    public static boolean IS_DISABLED = false;
    public static int HIGH_STOP = 750;
    public static int HIGH_LIMIT = 675;
    public static int LOW_LIMIT = 250;
    public static int LOW_STOP = 5;

    public enum ExtendMode {
        OPEN_LOOP,
        GOTO
    }

    public enum MaturicaMode {
        IN,
        IDLE,
        OUT
    }

    public enum CarutaMode {
        START,
        TRANSFER,
        FLY,
        COLLECT,
        DISABLE
    }

    public ExtendMode extendMode;
    public MaturicaMode maturicaMode;
    public CarutaMode carutaMode;

    private DcMotorEx maturicaMotor;
    private DcMotor extendMotor;
    private Servo carutaStanga;
    private Servo carutaDreapta;
    private Robot robot;
    private double extendPower;
    private static int startPosition;

    Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        maturicaMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "maturicaMotor"));
        extendMotor = hardwareMap.get(DcMotor.class, "maturicaExtendMotor");

        carutaStanga = hardwareMap.get(Servo.class, "carutaLeft");
        carutaDreapta = hardwareMap.get(Servo.class, "carutaRight");

        extendMotor.setDirection(DcMotorEx.Direction.REVERSE);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        maturicaMode = MaturicaMode.IDLE;
        extendMode = ExtendMode.OPEN_LOOP;
        extendPower = 0;
        carutaMode = CarutaMode.START;
    }

    public void resetExtend() {
        startPosition = robot.getRevBulkDataHub1().getMotorCurrentPosition(extendMotor);
    }

    public void toggleDisable() {
        if (carutaMode != CarutaMode.DISABLE) {
            carutaMode = CarutaMode.DISABLE;
            maturicaMode = MaturicaMode.IN;
            carutaStanga.getController().pwmDisable();
        } else {
            carutaMode = CarutaMode.FLY;
            carutaStanga.getController().pwmEnable();
        }
    }

    public void setExtendPower(double extendPower) {
        this.extendPower = extendPower;
    }

    public int getExtendEncoder() {
        return robot.getRevBulkDataHub1().getMotorCurrentPosition(extendMotor) - startPosition;
    }

    public void goToPositionExtend(int pos) {
        if (extendMode != ExtendMode.GOTO) {
            extendMode = ExtendMode.GOTO;
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.sleep(0.01);
        }
        extendMotor.setTargetPosition(pos);
        extendMotor.setPower(1);
    }

    @Override
    public void update() {
        if (IS_DISABLED)
            return;

        switch (maturicaMode) {
            case IN:
                maturicaMotor.setPower(0.9);
                break;
            case IDLE:
                maturicaMotor.setPower(0);
                break;
            case OUT:
                maturicaMotor.setPower(-0.5);
                break;
        }

        switch (carutaMode) {
            case START:
                carutaStanga.setPosition(0.575);
                carutaDreapta.setPosition(0.425);
                break;
            case TRANSFER:
                carutaStanga.setPosition(0.45); //55
                carutaDreapta.setPosition(0.55); //435
                break;
            case FLY:
                carutaStanga.setPosition(0.140);//100
                carutaDreapta.setPosition(0.860);
                break;
            case COLLECT:
                carutaStanga.setPosition(0);
                carutaDreapta.setPosition(0.985);
                break;
            case DISABLE:
                break;
        }

        switch (extendMode) {
            case OPEN_LOOP:
                if (extendPower < 0.01 && extendPower > -0.01)
                    extendMotor.setPower(0);
                else if (extendPower > 0) {
                    if (getExtendEncoder() < HIGH_LIMIT)
                        extendMotor.setPower(extendPower);
                    else if (HIGH_LIMIT < getExtendEncoder() && getExtendEncoder() <= HIGH_STOP)
                        extendMotor.setPower(extendPower * 0.3);
                    else
                        extendMotor.setPower(0);
                } else {
                    if (getExtendEncoder() > LOW_LIMIT)
                        extendMotor.setPower(extendPower);
                    else if (LOW_LIMIT > getExtendEncoder() && getExtendEncoder() >= LOW_STOP)
                        extendMotor.setPower(extendPower * 0.45);
                    else
                        extendMotor.setPower(0);
                }
                break;
            case GOTO:
                break;
        }
    }
}
