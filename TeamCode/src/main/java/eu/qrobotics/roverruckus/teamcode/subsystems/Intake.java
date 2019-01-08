package eu.qrobotics.roverruckus.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@Config
public class Intake implements Subsystem {

    public enum MaturicaMode {
        IN,
        IDLE,
        OUT
    }

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


    public MaturicaMode maturicaMode;
    public CarutaMode carutaMode;
    public ExtendMode extendMode;

    private ExpansionHubMotor maturicaMotor;
    private ExpansionHubMotor extendMotor;
    private ExpansionHubServo carutaStanga;
    private ExpansionHubServo carutaDreapta;

    Intake(HardwareMap hardwareMap) {
        maturicaMotor = hardwareMap.get(ExpansionHubMotor.class, "maturicaMotor");
        extendMotor = hardwareMap.get(ExpansionHubMotor.class, "maturicaExtendMotor");

        carutaStanga = hardwareMap.get(ExpansionHubServo.class, "carutaLeft");
        carutaDreapta = hardwareMap.get(ExpansionHubServo.class, "carutaRight");

        maturicaMotor.setDirection(ExpansionHubMotor.Direction.REVERSE);
        extendMotor.setDirection(ExpansionHubMotor.Direction.REVERSE);

        //maturicaMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);

        maturicaMode = MaturicaMode.IDLE;
        extendMode = ExtendMode.IDLE;
        carutaMode = CarutaMode.UP;
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
                extendMotor.setPower(0.50);
                break;
            case BACK:
                extendMotor.setPower(-0.50);
                break;
        }
    }
}
