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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import eu.qrobotics.roverruckus.teamcode.util.AxesSigns;
import eu.qrobotics.roverruckus.teamcode.util.BNO055IMUUtil;
import eu.qrobotics.roverruckus.teamcode.util.LynxOptimizedI2cFactory;
import eu.qrobotics.roverruckus.teamcode.util.MecanumUtil;

@Config
public class MecanumDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive implements Subsystem {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
    public static boolean USE_EXTERNAL_HEADING = true;
    public static boolean USE_CACHING = false;
    public static boolean IS_DISABLED = false;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private Robot robot;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    // Caching stuff
    private double[] powers;
    private boolean isWheelPositionCached = false;
    private List<Double> cachedWheelPositions = null;
    private boolean isHeadingCached = false;
    private double cachedHeading = 0;

    MecanumDrive(HardwareMap hwMap, Robot robot) {
        super(DriveConstants.TRACK_WIDTH);

        this.robot = robot;

        constraints = new MecanumConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        follower = new MecanumPIDVAFollower(this, TRANSLATIONAL_PID, HEADING_PID,
                DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);
        setLocalizer(new MecanumDrive.MecanumLocalizer(this, USE_EXTERNAL_HEADING));

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(robot.getHub1().getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        //MARK: Remap IMU axes for vertical hub
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hwMap.get(ExpansionHubMotor.class, "leftFront");
        leftRear = hwMap.get(ExpansionHubMotor.class, "leftRear");
        rightRear = hwMap.get(ExpansionHubMotor.class, "rightRear");
        rightFront = hwMap.get(ExpansionHubMotor.class, "rightFront");

        rightRear.setDirection(ExpansionHubMotor.Direction.REVERSE);
        rightFront.setDirection(ExpansionHubMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        powers = new double[4];
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
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
        wheelPositions.add(DriveConstants.encoderTicksToInches(robot.getRevBulkDataHub1().getMotorCurrentPosition(leftFront)));
        wheelPositions.add(DriveConstants.encoderTicksToInches(robot.getRevBulkDataHub2().getMotorCurrentPosition(leftRear)));
        wheelPositions.add(DriveConstants.encoderTicksToInches(robot.getRevBulkDataHub2().getMotorCurrentPosition(rightRear)));
        wheelPositions.add(DriveConstants.encoderTicksToInches(robot.getRevBulkDataHub1().getMotorCurrentPosition(rightFront)));
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
        if (isFollowingTrajectory()) {
            updatePoseEstimate();
            updateFollower();
            if (USE_CACHING)
                internalSetMotorPowers();
        } else
            internalSetMotorPowers();
    }
}
