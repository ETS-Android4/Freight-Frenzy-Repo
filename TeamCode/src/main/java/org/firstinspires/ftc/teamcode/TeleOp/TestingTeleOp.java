package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass;
import org.firstinspires.ftc.teamcode.TurretClasses.RotateClass;
import org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass;

//@TeleOp
public class TestingTeleOp extends LinearOpMode {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
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

            //getting exponential joystick control for the drivetrain
            x = -(Math.copySign(gamepad1.left_stick_x, gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x));
            y = -(Math.copySign(gamepad1.left_stick_y, gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y));
            z = Math.copySign(gamepad1.right_stick_x, gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

            //setting the possiblity of a slow speed on the drivetrain
            if(gamepad1.right_bumper){
                robot.LF_M.setPower(.4*((y)-x+(z)));//LF
                robot.LB_M.setPower(.4*((y)+x+(z)));//LB
                robot.RF_M.setPower(.4*(-((y)+x-(z))));//RF
                robot.RB_M.setPower(.4*(-((y)-x-(z))));//RB
            }else{
                robot.LF_M.setPower(.8*((y)-x+(.8*z)));//LF
                robot.LB_M.setPower(.8*((y)+x+(.8*z)));//LB
                robot.RF_M.setPower(.8*(-((y)+x-(.8*z))));//RF
                robot.RB_M.setPower(.8*(-((y)-x-(.8*z))));//RB
            }

            if(gamepad1.a || gamepad2.a){
                robot.RI_S.setPower(-1);
                robot.LI_S.setPower(1);
            }else if(gamepad1.b || gamepad2.b){
                robot.RI_S.setPower(.5);
                robot.LI_S.setPower(-.5);
            }else{
                robot.RI_S.setPower(0);
                robot.LI_S.setPower(0);
            }



                Vpivotcontroller = Vpivotcontroller +(.03 * gamepad2.right_stick_y);
                //robot.TP_M.setPower(VPivotClass.VPivotMethod(Vpivotcontroller ,robot.TP_P.getVoltage()));
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
