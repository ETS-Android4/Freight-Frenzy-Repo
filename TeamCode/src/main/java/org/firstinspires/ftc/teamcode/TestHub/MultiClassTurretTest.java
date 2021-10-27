package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RotateClass;

@TeleOp
public class MultiClassTurretTest extends LinearOpMode {
    TestHubHardware robot = new TestHubHardware();
    ExtendClass ExtendClass = new ExtendClass();
    VPivotClass VPivotClass = new VPivotClass();
    RotateClass RotateClass = new RotateClass();


    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {


            robot.Motor1.setPower(ExtendClass.ExtendMethod(gamepad1.left_stick_y, robot.Motor1.getCurrentPosition(), robot.MagneticExtend.getState()));
            robot.Motor2.setPower(VPivotClass.VPivotMethod(gamepad1.right_stick_y, robot.PivotPT.getVoltage()));
            robot.Motor3.setPower(RotateClass.RotateMethod(gamepad1.right_stick_x, robot.Motor3.getCurrentPosition(), robot.MagneticRotate.getState()));


            telemetry.update();
        }
    }
}
