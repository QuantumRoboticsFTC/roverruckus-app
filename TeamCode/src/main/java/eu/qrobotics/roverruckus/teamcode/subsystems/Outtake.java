package eu.qrobotics.roverruckus.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

public class Outtake implements Subsystem {

    public enum LiftMode {
        UP,
        IDLE,
        DOWN
    }

    public enum DumpMode {
        DOWN,
        UP
    }

    public LiftMode liftMode;
    public DumpMode dumpMode;

    private ExpansionHubMotor liftMotor;
    private ExpansionHubServo leftDump;
    private ExpansionHubServo rightDump;

    public Outtake(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(ExpansionHubMotor.class, "liftMotor");
        leftDump = hardwareMap.get(ExpansionHubServo.class, "leftDump");
        rightDump = hardwareMap.get(ExpansionHubServo.class, "rightDump");

        //liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //liftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        liftMode = LiftMode.IDLE;
        dumpMode = DumpMode.DOWN;
    }

    @Override
    public void update() {
        switch (liftMode) {
            case IDLE:
                liftMotor.setPower(0.05);
                break;
            case UP:
                liftMotor.setPower(1);
                break;
            case DOWN:
                liftMotor.setPower(-0.5);
                break;
        }

        switch (dumpMode) {
            case DOWN:
                leftDump.setPosition(0.85);
                rightDump.setPosition(0.144);
                break;
            case UP:
                leftDump.setPosition(0.149); //0.449
                rightDump.setPosition(0.9439); //0.5439
                break;
        }
    }
}
