package eu.qrobotics.roverruckus.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import eu.qrobotics.roverruckus.teamcode.hardware.CachingDcMotorEx;

/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class Climb implements Subsystem {
    private static final double TICKS_PER_REV = 103.6;

    public static boolean IS_DISABLED = false;

    public static double SPOOL_RADIUS = 0.69; // in
    public static double GEAR_RATIO = 1.0 / 24.0; // output/input

    // the operating range of the elevator is restricted to [0, MAX_HEIGHT]
    public static double MAX_HEIGHT = 9.5; // 8.85 in

    public static PIDCoefficients PID = new PIDCoefficients(2, 0, 0);

    public static double MAX_VEL = 4; // in/s
    public static double MAX_ACCEL = 7; // in/s^2
    public static double MAX_JERK = 10; // in/s^3

    public static double kV = 0.20515;
    public static double kA = -0.00026;
    public static double kStatic = 0.11539;
    public double globalPower = 0;

    private Robot robot;
    private DcMotorEx motor;
    private PIDFController controller;
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredHeight = 0;
    private int offset;
    private boolean autonomous = false;


    private static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return 1620;
    }

    public Climb(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        motor = hardwareMap.get(DcMotorEx.class, "climbMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // if necessary, reverse the motor so "up" is positive
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // note: if the elevator is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it here (assuming no velocity PID) like so:
        // e.g., controller = new PIDFController(PID, kV, kA, kStatic, x -> kA * 9.81);
        controller = new PIDFController(PID, kV, kA, kStatic);
        offset = robot.getRevBulkDataHub1().getMotorCurrentPosition(motor);
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public void setAutonomous() {
        autonomous = !autonomous;
    }

    public void setHeight(double height) {
        height = Math.min(Math.max(0, height), MAX_HEIGHT);

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(desiredHeight, 0, 0, 0);
        MotionState goal = new MotionState(height, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );
        profileStartTime = clock.seconds();

        this.desiredHeight = height;
    }

    public double getCurrentHeight() {
        return encoderTicksToInches(robot.getRevBulkDataHub1().getMotorCurrentPosition(motor) - offset);
    }

    public void update() {
        if (IS_DISABLED)
            return;
        if (autonomous) {
            double power;
            double currentHeight = getCurrentHeight();
            if (isBusy()) {
                // following a profile
                double time = clock.seconds() - profileStartTime;
                MotionState state = profile.get(time);
                controller.setTargetPosition(state.getX());
                power = controller.update(currentHeight, state.getV(), state.getA());
            } else {
                // just hold the position
                controller.setTargetPosition(desiredHeight);
                power = controller.update(currentHeight);
            }
            setPower(power);
        } else
            setPower(globalPower);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

}