package eu.qrobotics.roverruckus.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import eu.qrobotics.roverruckus.teamcode.util.StickyGamepad;

@TeleOp(name = "Double Servo Programmer", group = "Test")
public class DoubleServoProgrammer extends OpMode {
    enum ProgrammerMode {
        Low, Medium, High;

        public double getRawValue() {
            switch (this) {
                case Low:
                    return 0.001;
                case Medium:
                    return 0.025;
                case High:
                    return 0.05;
                default:
                    return 0.05;
            }
        }

        public ProgrammerMode nextMode() {
            switch (this) {
                case Low:
                    return ProgrammerMode.Medium;
                case Medium:
                    return ProgrammerMode.High;
                case High:
                    return ProgrammerMode.Low;
                default:
                    return ProgrammerMode.High;

            }
        }

        public String stringValue() {
            switch (this) {
                case Low:
                    return "Low";
                case Medium:
                    return "Medium";
                case High:
                    return "High";
                default:
                    return "Unknown";
            }
        }
    }

    private Servo leftServo = null;
    private Servo rightServo = null;

    private StickyGamepad stickyGamepad = null;

    private boolean isLeftServoDisabled = true;
    private boolean isRightServoDisabled = true;

    private ProgrammerMode programmerMode = ProgrammerMode.High;

    // initial servo positions
    private double currentPositionLeft = 0.935;
    private double currentPositionRight = 0.065;

    private Servo scorpionLeft = null;
    private Servo scorpionRight = null;

    @Override
    public void init() {

        scorpionLeft = hardwareMap.get(Servo.class, "carutaLeft");
        scorpionRight = hardwareMap.get(Servo.class, "carutaRight");

        scorpionLeft.setPosition(0.55);
        scorpionRight.setPosition(0.435);

        leftServo = hardwareMap.get(Servo.class, "leftScorpion");
        rightServo = hardwareMap.get(Servo.class, "rightScorpion");

        stickyGamepad = new StickyGamepad(gamepad1);

        telemetry.addData("Initial Left Servo Position", currentPositionLeft);
        telemetry.addData("Initial Right Servo Position", currentPositionRight);
    }

    @Override
    public void loop() {
        stickyGamepad.update();

        // set precision
        if (stickyGamepad.x) {
            programmerMode = programmerMode.nextMode();
        }

        // set position to both servos
        if (stickyGamepad.right_bumper) {
            currentPositionLeft -= programmerMode.getRawValue();
            currentPositionRight += programmerMode.getRawValue();
        } else if (stickyGamepad.left_bumper) {
            currentPositionLeft += programmerMode.getRawValue();
            currentPositionRight -= programmerMode.getRawValue();
        }

        updateLeftServo();
        updateRightServo();

        updateTelemetry();
    }

    private void updateLeftServo() {
        // set servo position
        if (stickyGamepad.dpad_down && currentPositionLeft >= programmerMode.getRawValue()) {
            currentPositionLeft -= programmerMode.getRawValue();
        } else if (stickyGamepad.dpad_up && currentPositionLeft + programmerMode.getRawValue() <= 1) {
            currentPositionLeft += programmerMode.getRawValue();
        }

        // toggle pwm
        if (stickyGamepad.dpad_right) {
            isLeftServoDisabled = !isLeftServoDisabled;

            // update pwm and servo position
            if (isLeftServoDisabled) {
                leftServo.getController().pwmDisable();
            } else {
                leftServo.getController().pwmEnable();
            }
        }

        leftServo.setPosition(currentPositionLeft);
    }

    private void updateRightServo() {
        // set servo position
        if (stickyGamepad.a && currentPositionRight >= programmerMode.getRawValue()) {
            currentPositionRight -= programmerMode.getRawValue();
        } else if (stickyGamepad.y && currentPositionRight + programmerMode.getRawValue() <= 1) {
            currentPositionRight += programmerMode.getRawValue();
        }

        // toggle pwm
        if (stickyGamepad.b) {
            isRightServoDisabled = !isRightServoDisabled;

            // update pwm and servo position
            if (isRightServoDisabled) {
                rightServo.getController().pwmDisable();
            } else {
                rightServo.getController().pwmEnable();

            }
        }

        rightServo.setPosition(currentPositionRight);
    }

    private void updateTelemetry() {
        telemetry.addData("Programmer Mode", programmerMode.stringValue());
        telemetry.addData("Precision", programmerMode.getRawValue());
        telemetry.addData("Left Servo Position", leftServo.getPosition());
        telemetry.addData("Right Servo Position", rightServo.getPosition());
        telemetry.addData("Left Servo Running", !isLeftServoDisabled);
        telemetry.addData("Right Servo Running", !isRightServoDisabled);
        telemetry.update();
    }

}

