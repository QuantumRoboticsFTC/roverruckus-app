package eu.qrobotics.roverruckus.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "HodlTest")
@Disabled
public class HodlTest extends OpMode {

    DcMotor motor = null;
    boolean lastA = false;
    boolean state = false;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.a && !lastA) {
            if (state)
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            else
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            state = !state;
        }
        lastA = gamepad1.a;
        if (gamepad1.b)
            motor.setPower(0.05);
        else
            motor.setPower(0);
        telemetry.addData("State", motor.getMode());
        telemetry.addData("Power", motor.getPower());
        telemetry.update();
    }
}
