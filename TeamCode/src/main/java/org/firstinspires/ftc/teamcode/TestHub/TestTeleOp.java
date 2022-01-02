package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass;
import org.firstinspires.ftc.teamcode.TurretClasses.RotateClass;
import org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    TurretTesting robot = new TurretTesting();
    org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass ExtendClass = new ExtendClass();
    org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass VPivotClass = new VPivotClass();
    RotateClass RotateClass = new RotateClass();
    double x, y, z;
    double Vpivotcontroller = 1.15;
    double initPOsitionOrder = 1;
    public void runOpMode() {
        robot.init(hardwareMap);


        waitForStart();
        while (opModeIsActive()) {



                    //else run TeleOp programs
                Vpivotcontroller = Vpivotcontroller +(.03 * gamepad2.right_stick_y);
             //   robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(Vpivotcontroller ,1,robot.TP_P.getVoltage()));
                // robot.TP_M.setPower(VPivotClass.VPivotMethod(gamepad2.right_stick_y, robot.TP_P.getVoltage()));
                robot.TR_M.setPower(RotateClass.RotateMethod(gamepad2.right_trigger, gamepad2.left_trigger, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));

                robot.TE_M.setPower(ExtendClass.ExtendMethod((-gamepad2.left_stick_y), robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));






            telemetry.addData("rotate motor power",robot.TR_M.getPower());
            telemetry.addData("rotate motor power", RotateClass.rotatemotorPowerReturn());
            telemetry.addData("extend motor power",robot.TE_M.getPower());
            telemetry.addData("pivot motor power", robot.TP_M.getPower());
            telemetry.addData("vpivotset", VPivotClass.PivotSetReturn());
            telemetry.addData("pivot POT", robot.TP_P.getVoltage());

            telemetry.update();
        }
    }
}
