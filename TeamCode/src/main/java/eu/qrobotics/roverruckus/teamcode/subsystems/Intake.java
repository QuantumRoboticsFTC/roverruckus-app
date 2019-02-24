package eu.qrobotics.roverruckus.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import eu.qrobotics.roverruckus.teamcode.hardware.CachingDcMotorEx;
import eu.qrobotics.roverruckus.teamcode.hardware.CachingServo;

@Config
public class Intake implements Subsystem {
    public static boolean IS_DISABLED = false;

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

    public MaturicaMode maturicaMode;
    public CarutaMode carutaMode;

    private DcMotorEx maturicaMotor;
    private DcMotorEx extendMotor;
    private Servo carutaStanga;
    private Servo carutaDreapta;
    private Robot robot;
    private double extendPower;
    private int startPosition;

    Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        maturicaMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "maturicaMotor"));
        extendMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "maturicaExtendMotor"));

        carutaStanga = new CachingServo(hardwareMap.get(Servo.class, "carutaLeft"));
        carutaDreapta = new CachingServo(hardwareMap.get(Servo.class, "carutaRight"));

        extendMotor.setDirection(DcMotorEx.Direction.REVERSE);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        startPosition = robot.getRevBulkDataHub1().getMotorCurrentPosition(extendMotor);
        maturicaMode = MaturicaMode.IDLE;
        extendPower = 0;
        carutaMode = CarutaMode.START;
    }

    public void toggleDisable() {
        if (carutaMode != CarutaMode.DISABLE) {
            carutaMode = CarutaMode.DISABLE;
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
                carutaStanga.setPosition(0.68);
                carutaDreapta.setPosition(0.305);
                break;
            case TRANSFER:
                carutaStanga.setPosition(0.55);
                carutaDreapta.setPosition(0.435);
                break;
            case FLY:
                carutaStanga.setPosition(0.2);
                carutaDreapta.setPosition(0.785);
                break;
            case COLLECT:
                carutaStanga.setPosition(0);
                carutaDreapta.setPosition(0.985);
                break;
            case DISABLE:
                break;
        }

        if (extendPower > 0 || (extendPower < 0 && getExtendEncoder() > 10))
            extendMotor.setPower(extendPower);
        else
            extendMotor.setPower(0);
    }
}
