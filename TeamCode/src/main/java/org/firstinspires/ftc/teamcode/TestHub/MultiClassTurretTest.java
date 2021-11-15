package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RotateClass;

@TeleOp
public class MultiClassTurretTest extends LinearOpMode {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    ExtendClass ExtendClass = new ExtendClass();
    VPivotClass VPivotClass = new VPivotClass();
    RotateClass RotateClass = new RotateClass();
    double x, y, z;

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

            if(gamepad2.x){
                robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15,.8,robot.TP_P.getVoltage()));
                if(robot.TP_P.getVoltage() > 1.05 && robot.TP_P.getVoltage() < 1.25){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0,.8, robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                    robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(300, .5, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                }
                if(robot.TE_M.getCurrentPosition() > 290 && robot.TE_M.getCurrentPosition() < 310 && RotateClass.modifiedRotateCurrent() < 50 && RotateClass.modifiedRotateCurrent() > -50){
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(.8,.8,robot.TP_P.getVoltage()));
                }

            }else{
                robot.TR_M.setPower(RotateClass.RotateMethod(gamepad2.right_trigger, gamepad2.left_trigger, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                robot.TP_M.setPower(VPivotClass.VPivotMethod(gamepad2.right_stick_y, robot.TP_P.getVoltage()));
                robot.TE_M.setPower(ExtendClass.ExtendMethod((-gamepad2.left_stick_y), robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
            }



            telemetry.addData("rotate motor power",robot.TR_M.getPower());
            telemetry.addData("extend motor power",robot.TE_M.getPower());
            telemetry.addData("pivot motor power", robot.TP_M.getPower());
            telemetry.addData("vpivotset", VPivotClass.PivotSetReturn());
            telemetry.addData("pivot POT", robot.TP_P.getVoltage());

            telemetry.update();
        }
    }
}
