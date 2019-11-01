package eu.qrobotics.roverruckus.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(group = "Test")
@Config
@Disabled
public class RunToPositionTuner extends LinearOpMode {
    public static double P = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        DcMotorEx extend = hardwareMap.get(DcMotorEx.class, "maturicaExtendMotor");
        extend.setPositionPIDFCoefficients(P);

        extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extend.setDirection(DcMotorEx.Direction.REVERSE);
        extend.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extend.setTargetPosition(0);
        extend.setPower(0.7);

        Servo carutaStanga = hardwareMap.get(Servo.class, "carutaLeft");
        Servo carutaDreapta = hardwareMap.get(Servo.class, "carutaRight");
        carutaStanga.setPosition(0.140);
        carutaDreapta.setPosition(0.860);


        double currentP = P;

        dashboard.updateConfig();

        RobotLog.i("Initial motor PID coefficients: " + P);

        telemetry.log().add("Ready!");
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
        if (isStopRequested())
            return;

        double lastRuntime = -20;

        while (!isStopRequested()) {
            // update the coefficients if necessary
            if (P != currentP) {
                RobotLog.i("Updated motor PID coefficients: " + P);
                currentP = P;
                extend.setPositionPIDFCoefficients(P);
            }

            if (getRuntime() - lastRuntime > 4) {
                extend.setTargetPosition((int) (50 + 500 * Math.random()));
                lastRuntime = getRuntime();
            }

            telemetry.addData("target extend", extend.getTargetPosition());
            telemetry.addData("extend encoder", extend.getCurrentPosition());
            telemetry.addData("error", extend.getTargetPosition() - extend.getCurrentPosition());
            telemetry.update();
        }
    }
}
