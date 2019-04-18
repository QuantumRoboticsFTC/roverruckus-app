package eu.qrobotics.roverruckus.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import eu.qrobotics.roverruckus.teamcode.subsystems.Intake;
import eu.qrobotics.roverruckus.teamcode.subsystems.Robot;

@Autonomous(group = "Test")
public class GoToPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true);
        robot.intake.carutaMode = Intake.CarutaMode.FLY;

        waitForStart();
        robot.start();
        robot.sleep(0.1);
        robot.intake.resetExtend();
        robot.intake.goToPositionExtend(500, 1);

        while (!isStopRequested() && opModeIsActive()) {
            idle();
        }
        robot.stop();
    }
}
