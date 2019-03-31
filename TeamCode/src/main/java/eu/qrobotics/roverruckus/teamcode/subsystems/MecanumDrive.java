package eu.qrobotics.roverruckus.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import eu.qrobotics.roverruckus.teamcode.hardware.CachingDcMotorEx;
import eu.qrobotics.roverruckus.teamcode.util.LynxOptimizedI2cFactory;
import eu.qrobotics.roverruckus.teamcode.util.MecanumUtil;

@Config
public class MecanumDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive implements Subsystem {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(1.3, 0, 0.4);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(2.54, 0, 0.5);
    public static boolean USE_EXTERNAL_HEADING = true;
    public static boolean USE_CACHING = false;
    public static boolean IS_DISABLED = false;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private Robot robot;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    // Caching stuff
    private double[] powers;
    private boolean isWheelPositionCached = false;
    private List<Double> cachedWheelPositions = null;
    private boolean isHeadingCached = false;
    private double cachedHeading = 0;
    private boolean autonomous = false;

    MecanumDrive(HardwareMap hwMap, Robot robot, boolean isAutonomous) {
        super(DriveConstants.TRACK_WIDTH);

        this.robot = robot;

        constraints = new MecanumConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        follower = new MecanumPIDVAFollower(this, TRANSLATIONAL_PID, HEADING_PID,
                DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);
        setLocalizer(new MecanumDrive.MecanumLocalizer(this, USE_EXTERNAL_HEADING));

        if (isAutonomous) {
            imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(robot.getHub1().getStandardModule(), 0);
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }

        //MARK: Remap IMU axes for vertical hub
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = new CachingDcMotorEx(hwMap.get(DcMotorEx.class, "leftFront"));
        leftRear = new CachingDcMotorEx(hwMap.get(DcMotorEx.class, "leftRear"));
        rightRear = new CachingDcMotorEx(hwMap.get(DcMotorEx.class, "rightRear"));
        rightFront = new CachingDcMotorEx(hwMap.get(DcMotorEx.class, "rightFront"));

        rightRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        powers = new double[4];
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void toggleAutonomous() {
        autonomous = !autonomous;
    }

    public Trajectory getTrajectory() {
        return follower.getTrajectory();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public boolean isFollowingTrajectory() {
        return follower.isFollowing();
    }

    public Pose2d getFollowingError() {
        return follower.getLastError();
    }

    //<editor-fold desc="Motor Power Caching">
    private void internalSetMotorPowers() {
        leftFront.setPower(powers[0]);
        leftRear.setPower(powers[1]);
        rightRear.setPower(powers[2]);
        rightFront.setPower(powers[3]);
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        powers[0] = v;
        powers[1] = v1;
        powers[2] = v2;
        powers[3] = v3;
        if (!USE_CACHING)
            internalSetMotorPowers();
    }
    //</editor-fold>

    //<editor-fold desc="Wheel Position Caching">
    private void internalGetWheelPositions() {
        isWheelPositionCached = true;

        if (robot.getRevBulkDataHub1() == null || robot.getRevBulkDataHub2() == null) {
            cachedWheelPositions = Arrays.asList(0.0, 0.0, 0.0, 0.0);
            return;
        }

        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(DriveConstants.encoderTicksToInches(robot.getRevBulkDataHub2().getMotorCurrentPosition(leftFront)));
        wheelPositions.add(DriveConstants.encoderTicksToInches(robot.getRevBulkDataHub2().getMotorCurrentPosition(leftRear)));
        wheelPositions.add(DriveConstants.encoderTicksToInches(robot.getRevBulkDataHub2().getMotorCurrentPosition(rightRear)));
        wheelPositions.add(DriveConstants.encoderTicksToInches(robot.getRevBulkDataHub2().getMotorCurrentPosition(rightFront)));
        cachedWheelPositions = wheelPositions;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        if (!USE_CACHING || !isWheelPositionCached)
            internalGetWheelPositions();
        return cachedWheelPositions;
    }
    //</editor-fold>

    //<editor-fold desc="Heading Caching">
    private void internalGetHeading() {
        isHeadingCached = true;
        cachedHeading = imu.getAngularOrientation().firstAngle;
    }

    @Override
    public double getExternalHeading() {
        if (!USE_CACHING || !isHeadingCached)
            internalGetHeading();
        return cachedHeading;
    }
    //</editor-fold>

    private void invalidateCache() {
        isWheelPositionCached = false;
        isHeadingCached = false;
    }

    public void updateFollower() {
        follower.update(getPoseEstimate());
    }

    public void setMotorsGamepad(Gamepad gg, double scale) {
        MecanumUtil.Wheels wh = MecanumUtil.motionToWheels(MecanumUtil.joystickToMotion(gg.left_stick_x, gg.left_stick_y,
                gg.right_stick_x, gg.right_stick_y)).scaleWheelPower(scale);
        powers[0] = wh.frontLeft;
        powers[1] = wh.backLeft;
        powers[2] = wh.backRight;
        powers[3] = wh.frontRight;
    }

    @Override
    public void update() {
        if (IS_DISABLED)
            return;

        invalidateCache();
        if (autonomous || isFollowingTrajectory()) {
            updatePoseEstimate();
            updateFollower();
            if (USE_CACHING)
                internalSetMotorPowers();
        } else
            internalSetMotorPowers();
    }
}
