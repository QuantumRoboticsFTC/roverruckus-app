package eu.qrobotics.roverruckus.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubServo;

import eu.qrobotics.roverruckus.teamcode.hardware.CachingDcMotorEx;
import eu.qrobotics.roverruckus.teamcode.hardware.CachingServo;

@Config
public class Outtake implements Subsystem {
    public static boolean IS_DISABLED = false;

    public enum ScorpionMode {
        DOWN,
        UP
    }

    public enum SorterMode {
        IN,
        CENTER,
        OUT
    }

    public enum DoorMode {
        CLOSE,
        STRAIGHT,
        OPEN
    }

    public ScorpionMode scorpionMode;
    public SorterMode sorterMode;
    public DoorMode doorMode;

    private DcMotorEx liftMotor;
    private Servo leftScorpion;
    private Servo rightScorpion;
    private Servo sorter;
    private Servo door;
    private DigitalChannel liftSwitch;
    private Robot robot;
    private double liftPower;
    private int startPosition;

    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        liftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftMotor"));
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        startPosition = robot.getRevBulkDataHub1().getMotorCurrentPosition(liftMotor);

        liftSwitch = hardwareMap.get(DigitalChannel.class, "liftSwitch");
        liftSwitch.setMode(DigitalChannel.Mode.INPUT);

        leftScorpion = hardwareMap.get(Servo.class, "leftScorpion");
        rightScorpion = hardwareMap.get(Servo.class, "rightScorpion");

        sorter = hardwareMap.get(Servo.class, "sorter");
        door = hardwareMap.get(Servo.class, "door");

        liftPower = 0;
        scorpionMode = ScorpionMode.DOWN;
        sorterMode = SorterMode.OUT;
        doorMode = DoorMode.OPEN;
    }

    public void setLiftPower(double liftPower) {
        this.liftPower = liftPower;
    }

    public int getLiftEncoder() {
        return robot.getRevBulkDataHub1().getMotorCurrentPosition(liftMotor) - startPosition;
    }

    public boolean isLiftUp() {
        return robot.getRevBulkDataHub1().getDigitalInputState(liftSwitch);
    }

    @Override
    public void update() {
        if (IS_DISABLED)
            return;

        if ((liftPower > 0 && !(isLiftUp()))
                || (liftPower < 0 && Math.abs(getLiftEncoder()) > 20))
            liftMotor.setPower(liftPower);
        else
            liftMotor.setPower(0.2);

        switch (scorpionMode) {
            case DOWN:
                leftScorpion.setPosition(0.965);//935
                rightScorpion.setPosition(0.035);//065
                break;
            case UP:
                leftScorpion.setPosition(0.2);
                rightScorpion.setPosition(0.8);
                break;
        }

        switch (sorterMode) {
            case IN:
                sorter.setPosition(0.125);
                break;
            case OUT:
                sorter.setPosition(0.325);
                break;
            case CENTER:
                sorter.setPosition(0.210);
                break;
        }

        switch (doorMode) {
            case OPEN:
                door.setPosition(0.23);
                break;
            case STRAIGHT:
                door.setPosition(0.12);
                break;
            case CLOSE:
                door.setPosition(0.025);
                break;
        }
    }
}
