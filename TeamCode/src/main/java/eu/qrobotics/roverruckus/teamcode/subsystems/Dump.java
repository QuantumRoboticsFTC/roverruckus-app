package eu.qrobotics.roverruckus.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Dump implements Subsystem {

    public enum ClimbMode {
        UP,
        IDLE,
        DOWN
    }

    public enum DumpMode {
        DOWN,
        UP
    }

    public ClimbMode climbMode;
    public DumpMode dumpMode;

    private DcMotorEx climbMotor;
    private Servo leftDump;
    private Servo rightDump;

    public Dump(HardwareMap hardwareMap) {
        climbMotor = hardwareMap.get(DcMotorEx.class, "climbMotor");
        leftDump = hardwareMap.get(Servo.class, "leftDump");
        rightDump = hardwareMap.get(Servo.class, "rightDump");

        //climbMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //climbMotor.setDirection(DcMotorEx.Direction.REVERSE);

        climbMode = ClimbMode.IDLE;
        dumpMode = DumpMode.DOWN;
    }

    @Override
    public void update() {
        switch (climbMode) {
            case IDLE:
                climbMotor.setPower(0.05);
                break;
            case UP:
                climbMotor.setPower(1);
                break;
            case DOWN:
                climbMotor.setPower(-0.5);
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
