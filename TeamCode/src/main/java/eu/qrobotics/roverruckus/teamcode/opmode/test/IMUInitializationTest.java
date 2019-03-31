package eu.qrobotics.roverruckus.teamcode.opmode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

import eu.qrobotics.roverruckus.teamcode.util.LynxOptimizedI2cFactory;

@Autonomous
public class IMUInitializationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevExtensions2.init();
        LynxModule module = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2").getStandardModule();
        LynxEmbeddedIMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        waitForStart();
        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(module, 0);
        imu.initialize(parameters);
        while (opModeIsActive()) {
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }

    }
}
