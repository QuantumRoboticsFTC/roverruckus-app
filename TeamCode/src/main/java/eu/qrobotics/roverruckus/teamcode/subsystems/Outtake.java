package eu.qrobotics.roverruckus.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

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

    private ExpansionHubMotor liftMotor;
    private ExpansionHubServo leftScorpion;
    private ExpansionHubServo rightScorpion;
    private ExpansionHubServo sorter;
    private ExpansionHubServo door;
    public DigitalChannel liftSwitch;
    private Robot robot;
    private double liftPower;
    private int startPosition;

    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        liftMotor = hardwareMap.get(ExpansionHubMotor.class, "liftMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        startPosition = robot.getRevBulkDataHub2().getMotorCurrentPosition(liftMotor);

        liftSwitch = hardwareMap.get(DigitalChannel.class, "liftSwitch");
        liftSwitch.setMode(DigitalChannel.Mode.INPUT);

        leftScorpion = hardwareMap.get(ExpansionHubServo.class, "leftScorpion");
        rightScorpion = hardwareMap.get(ExpansionHubServo.class, "rightScorpion");

        sorter = hardwareMap.get(ExpansionHubServo.class, "sorter");
        door = hardwareMap.get(ExpansionHubServo.class, "door");

        liftPower = 0;
        scorpionMode = ScorpionMode.DOWN;
        sorterMode = SorterMode.OUT;
        doorMode = DoorMode.OPEN;
    }

    public void setLiftPower(double liftPower) {
        this.liftPower = liftPower;
    }

    @Override
    public void update() {
        if (IS_DISABLED)
            return;

        if ((liftPower > 0 && !robot.getRevBulkDataHub2().getDigitalInputState(liftSwitch))
                || (liftPower < 0 && Math.abs(robot.getRevBulkDataHub2().getMotorCurrentPosition(liftMotor) - startPosition) > 20))
            liftMotor.setPower(liftPower);
//        else if (Math.abs(robot.getRevBulkDataHub2().getMotorCurrentPosition(liftMotor) - startPosition) <= 20)
//            liftMotor.setPower(0);
        else
            liftMotor.setPower(0.2);

        switch (scorpionMode) {
            case DOWN:
                leftScorpion.setPosition(0.925);//935
                rightScorpion.setPosition(0.075);//065
                break;
            case UP:
                leftScorpion.setPosition(0.15);
                rightScorpion.setPosition(0.85);
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
