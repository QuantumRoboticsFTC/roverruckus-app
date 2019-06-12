package eu.qrobotics.roverruckus.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import eu.qrobotics.roverruckus.teamcode.hardware.CachingDcMotorEx;

@Config
public class Intake implements Subsystem {
    public static double RUN_TO_POSITION_P = 17;

    public static boolean IS_DISABLED = false;
    public static int HIGH_STOP = 750;
    public static int HIGH_LIMIT = 675;
    public static int LOW_LIMIT = 300;
    public static int LOW_STOP = 5;

    // Subsystem state enums
    public enum ExtendMode {
        OPEN_LOOP,
        GOTO,
        SUICIDE
    }

    public enum MaturicaMode {
        IN,
        IDLE,
        OUT,
        FAST_OUT
    }

    public enum CarutaMode {
        START,
        TRANSFER,
        FLY,
        COLLECT,
        DISABLE
    }

    public enum DoorMode {
        OPEN,
        CLOSE_DEPOT,
        CLOSE
    }

    public ExtendMode extendMode;
    public MaturicaMode maturicaMode;
    public CarutaMode carutaMode;
    public DoorMode doorMode;

    private DcMotorEx maturicaMotor;
    private DcMotorEx extendMotor;
    private Servo carutaStanga;
    private Servo carutaDreapta;
    private Servo door;
    private Robot robot;
    private double extendPower;
    private static int startPosition;

    Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        maturicaMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "maturicaMotor"));
        maturicaMotor.setDirection(DcMotorEx.Direction.REVERSE);
        extendMotor = hardwareMap.get(DcMotorEx.class, "maturicaExtendMotor");

        carutaStanga = hardwareMap.get(Servo.class, "carutaLeft");
        carutaDreapta = hardwareMap.get(Servo.class, "carutaRight");

        door = hardwareMap.get(Servo.class, "intakeDoor");

        extendMotor.setDirection(DcMotorEx.Direction.REVERSE);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setPositionPIDFCoefficients(RUN_TO_POSITION_P);
        extendMotor.setTargetPositionTolerance(15);
        extendMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        maturicaMode = MaturicaMode.IDLE;
        extendMode = ExtendMode.OPEN_LOOP;
        extendPower = 0;
        carutaMode = CarutaMode.START;
        doorMode = DoorMode.OPEN;
    }

    public void resetExtend() {
        startPosition = robot.getRevBulkDataHub1().getMotorCurrentPosition(extendMotor);
    }

    public void toggleSuicide() {
        if (extendMode != ExtendMode.SUICIDE)
            extendMode = ExtendMode.SUICIDE;
        else {
            resetExtend();
            extendMode = ExtendMode.OPEN_LOOP;
        }
    }

    public void toggleDisable() {
        toggleDisable(false);
    }

    public void toggleDisable(boolean isDepot) {
        if (carutaMode != CarutaMode.DISABLE) {
            carutaMode = CarutaMode.DISABLE;
            maturicaMode = MaturicaMode.IN;
            if (isDepot)
                doorMode = DoorMode.CLOSE_DEPOT;
            else
                doorMode = DoorMode.CLOSE;
            carutaStanga.getController().pwmDisable();
        } else {
            carutaMode = CarutaMode.FLY;
            doorMode = DoorMode.OPEN;
            carutaStanga.getController().pwmEnable();
        }
    }

    public void setExtendPower(double extendPower) {
        this.extendPower = extendPower;
    }

    public int getExtendEncoder() {
        return robot.getRevBulkDataHub1().getMotorCurrentPosition(extendMotor) - startPosition;
    }

    public void goToPositionExtend(int pos, double speed) {
        if (extendMode != ExtendMode.GOTO) {
            extendMode = ExtendMode.GOTO;
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.sleep(0.01);
        }
        extendMotor.setTargetPosition(extendMotor.getCurrentPosition() + pos);
        extendMotor.setPower(speed);
    }

    public boolean isExtendAtTarget() {
        return robot.getRevBulkDataHub1().isMotorAtTargetPosition(extendMotor);
    }

    @Override
    public void update() {
        if (IS_DISABLED)
            return;

        switch (maturicaMode) {
            case IN:
                maturicaMotor.setPower(1);
                break;
            case IDLE:
                maturicaMotor.setPower(0);
                break;
            case OUT:
                maturicaMotor.setPower(-0.3);
                break;
            case FAST_OUT:
                maturicaMotor.setPower(-0.75);
                break;
        }

        switch (carutaMode) {
            case START:
                carutaStanga.setPosition(0.5);
                carutaDreapta.setPosition(0.5);
                break;
            case TRANSFER:
                carutaStanga.setPosition(0.4);
                carutaDreapta.setPosition(0.6);
                break;
            case FLY:
                carutaStanga.setPosition(0.1);
                carutaDreapta.setPosition(0.9);
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
                if (extendPower < 0.01 && extendPower > -0.01) // Dead-band
                    extendMotor.setPower(0);
                else if (extendPower > 0) { // Over-extension protection and speed "curve"
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
            case SUICIDE: // Mode used in case encoders are reset
                extendMotor.setPower(extendPower * 0.5);
                break;
        }

        switch (doorMode) {
            case OPEN:
                door.setPosition(0.18);
                break;
            case CLOSE:
                door.setPosition(0.85);
                break;
            case CLOSE_DEPOT:
                door.setPosition(0.87);
                break;
        }
    }
}
