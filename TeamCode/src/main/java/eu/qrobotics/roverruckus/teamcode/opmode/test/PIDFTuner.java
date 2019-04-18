package eu.qrobotics.roverruckus.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(group = "Test")
@Config
@Disabled
public class PIDFTuner extends LinearOpMode {
    public static PIDFCoefficients MOTOR_PID = new PIDFCoefficients();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        DcMotorEx extend = hardwareMap.get(DcMotorEx.class, "maturicaExtendMotor");

        extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extend.setDirection(DcMotorEx.Direction.REVERSE);
        extend.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extend.setTargetPosition(0);
        extend.setPower(0.7);

        Servo carutaStanga = hardwareMap.get(Servo.class, "carutaLeft");
        Servo carutaDreapta = hardwareMap.get(Servo.class, "carutaRight");
        carutaStanga.setPosition(0.140);
        carutaDreapta.setPosition(0.860);


        PIDFCoefficients currentCoeffs = extend.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);

        pidCopy(currentCoeffs, MOTOR_PID);
        dashboard.updateConfig();

        RobotLog.i("Initial motor PID coefficients: " + MOTOR_PID);

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
            if (!pidEquals(currentCoeffs, MOTOR_PID)) {
                RobotLog.i("Updated motor PID coefficients: " + MOTOR_PID);
                pidCopy(MOTOR_PID, currentCoeffs);
                extend.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, MOTOR_PID);
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
    private static boolean pidEquals(PIDFCoefficients coeff1, PIDFCoefficients coeff2) {
        return coeff1.p == coeff2.p && coeff1.i == coeff2.i && coeff1.d == coeff2.d && coeff1.f == coeff2.f && coeff1.algorithm == coeff2.algorithm;
    }

    private static void pidCopy(PIDFCoefficients source, PIDFCoefficients dest) {
        dest.p = source.p;
        dest.i = source.i;
        dest.d = source.d;
        dest.f = source.f;
        dest.algorithm = source.algorithm;
    }
}
