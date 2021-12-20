package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TurretClasses.RotateClass;
import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass;
import org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass;

@TeleOp
public class RedTeleOp extends LinearOpMode {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass ExtendClass = new ExtendClass();
    org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass VPivotClass = new VPivotClass();
    RotateClass RotateClass = new RotateClass();
    double x, y, z;
    double Vpivotcontroller = 1.15;
    double initPOsitionOrder = 1;
    public void runOpMode() {
        robot.init(hardwareMap);
        while (!opModeIsActive()) {
            if (RotateClass.isHomedRotateReturn() == false) {
                robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15, .5, robot.TP_P.getVoltage()));
                if (robot.TP_P.getVoltage() > 1.1 && robot.TP_P.getVoltage() < 1.25) {
                    robot.TE_M.setPower(ExtendClass.ExtendHoming(robot.TE_G.getState(), robot.TE_M.getCurrentPosition()));
                    if (ExtendClass.isHomedExtendReturn() == true) {
                        robot.TR_M.setPower(RotateClass.RotateHoming(robot.TR_G.getState(), robot.TR_M.getCurrentPosition()));
                    }
                }
            } else {
                telemetry.addData("homed", 0);
                robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(10, .8, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                if (initPOsitionOrder == 1) {
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0, .4, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                    if (RotateClass.modifiedRotateCurrent() > -50 && RotateClass.modifiedRotateCurrent() < 50) {
                        initPOsitionOrder = 2;
                    }
                } else if (initPOsitionOrder == 2) {
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15, .5, robot.TP_P.getVoltage()));
                    if (robot.TP_P.getVoltage() < 1.25 && robot.TP_P.getVoltage() > 1) {
                        initPOsitionOrder = 3;
                    }
                } else if (initPOsitionOrder == 3) {
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0, .4, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15, .5, robot.TP_P.getVoltage()));

                }


            }
            telemetry.addData("Rotate homed boolean", RotateClass.isHomedRotateReturn());
            telemetry.addData("initPosition order", initPOsitionOrder);
            telemetry.addData("Vpiovot PT", robot.TP_P.getVoltage());
            telemetry.addData("rotate modified", RotateClass.modifiedRotateCurrent());
            telemetry.update();
        }
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

            if(gamepad2.dpad_down){
                    //if arm is between deadzone of setpoint
                if(robot.TP_P.getVoltage() > 1.05 && robot.TP_P.getVoltage() < 1.25){
                        //rotate turret to "home"
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0,.8, robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                        //extend to 300 encoder ticks
                    robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(400, .5, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                }
                //look to see if previous task is completed
                if(ExtendClass.extendModifiedEncoder > 380 && ExtendClass.extendModifiedEncoder < 420 && RotateClass.modifiedRotateCurrent() < 50 && RotateClass.modifiedRotateCurrent() > -50){
                        //pivot arm down to intaking position
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.75,.6,robot.TP_P.getVoltage()));
                        //hold rotate position at "home"
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0,.8, robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                        //Hold extend at 300 encoder ticks
                    robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(400, .5, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                    Vpivotcontroller = VPivotClass.PivotSetReturn();
                }else{
                    //pivot to cleared height
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15,.8,robot.TP_P.getVoltage()));
                }

            }else if(gamepad2.dpad_up){
                robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.32,.7,robot.TP_P.getVoltage()));
                robot.TR_M.setPower(RotateClass.RotateMethod(gamepad2.right_trigger, gamepad2.left_trigger, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                robot.TE_M.setPower(ExtendClass.ExtendMethod((-gamepad2.left_stick_y), robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                Vpivotcontroller = VPivotClass.PivotSetReturn();

            }else if(gamepad2.dpad_right){
                robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15,.9,robot.TP_P.getVoltage()));
                Vpivotcontroller = VPivotClass.PivotSetReturn();
                robot.TR_M.setPower(RotateClass.RotateAutoMethod(-1400,.8, robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                robot.TE_M.setPower(ExtendClass.ExtendMethod((-gamepad2.left_stick_y), robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
            }else{
                    //else run TeleOp programs
                Vpivotcontroller = Vpivotcontroller +(.03 * gamepad2.right_stick_y);
                robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(Vpivotcontroller ,1,robot.TP_P.getVoltage()));
                // robot.TP_M.setPower(VPivotClass.VPivotMethod(gamepad2.right_stick_y, robot.TP_P.getVoltage()));
                robot.TR_M.setPower(RotateClass.RotateMethod(gamepad2.right_trigger, gamepad2.left_trigger, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));

                robot.TE_M.setPower(ExtendClass.ExtendMethod((-gamepad2.left_stick_y), robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
            }





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
