package eu.qrobotics.roverruckus.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

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

    private ExpansionHubMotor maturicaMotor;
    private ExpansionHubMotor extendMotor;
    private ExpansionHubServo carutaStanga;
    private ExpansionHubServo carutaDreapta;
    public DigitalChannel extendSwitch;
    private Robot robot;
    private double extendPower;
    private int startPosition;

    Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        maturicaMotor = hardwareMap.get(ExpansionHubMotor.class, "maturicaMotor");
        extendMotor = hardwareMap.get(ExpansionHubMotor.class, "maturicaExtendMotor");

        carutaStanga = hardwareMap.get(ExpansionHubServo.class, "carutaLeft");
        carutaDreapta = hardwareMap.get(ExpansionHubServo.class, "carutaRight");

        extendSwitch = hardwareMap.get(DigitalChannel.class, "extendSwitch");

        //maturicaMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        extendMotor.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);

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

    //TODO: Maturica analog
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

        if (extendPower > 0 || (extendPower < 0 && Math.abs(robot.getRevBulkDataHub1().getMotorCurrentPosition(extendMotor) - startPosition) > 5))
            extendMotor.setPower(extendPower);
        else
            extendMotor.setPower(0);
    }
}
