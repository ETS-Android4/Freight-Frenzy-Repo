package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass;
import org.firstinspires.ftc.teamcode.TurretClasses.RotateClass;
import org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass;


@Config
@TeleOp
public class BlueTeleOp extends LinearOpMode{

    public static double UPARMPM = .015;
    public static double UPARMDM = .009;
    public static double DNPM = .004;
    public static double DNDM = .012;
    public  static double SPEEDSET = 16;
    public static double MINSPEED = .2;
    public static double SETPOINT = 1500;
    public static double ROTATESPEED = 1;
    public static double VPIVOTSPEED = 16;
    public static double EXTENDSPEED = 1;


    public double vPivotSetPoint = 1500, extendSetPoint = 0, rotateSet = 0;
    public double x, y, z, initPOsitionOrder = 1;


    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    VPivotClass VPivotClass = new VPivotClass();
    ExtendClass ExtendClass = new ExtendClass();
    RotateClass RotateClass = new RotateClass();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();



    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

    /*    while (!opModeIsActive()){
            if(RotateClass.isHomedRotateReturn() == false){
                if(VPivotClass.has1stloop == true){
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(2000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));
                    if(VPivotClass.encoderWithOffset > 1900){
                        robot.TE_M.setPower(ExtendClass.ExtendHoming(robot.TE_G.getState(), robot.TE_M.getCurrentPosition()));
                        if(ExtendClass.isHomedExtendReturn() == true){
                            robot.TR_M.setPower(RotateClass.RotateHoming(robot.TR_G.getState(), robot.TR_M.getCurrentPosition()));
                        }
                    }
                }else{
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(3000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));
                }

            }else{
                telemetry.addData("homed",0);
                robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(10,.8,robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                if(initPOsitionOrder == 1){
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(2000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0,.8,robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                    if(RotateClass.modifiedRotateCurrent() > 750 && RotateClass.modifiedRotateCurrent() < 850){
                        initPOsitionOrder = 2;
                    }
                }



            }
            telemetry.addData("Rotate homed boolean", RotateClass.isHomedRotateReturn());
            telemetry.addData("initPosition order", initPOsitionOrder);
            telemetry.addData("Vpiovot PT", robot.TP_P.getVoltage());
            telemetry.addData("rotate modified", RotateClass.modifiedRotateCurrent());
            telemetry.update();
        }*/





        waitForStart();

        while (opModeIsActive()){

            /*
            if(gamepad2.dpad_down){
                if(VPivotClass.encoderWithOffset < 500){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0, .7, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                    rotateSet = 0;
                    robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(400, .7, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                    extendSetPoint = 400;

                    if(RotateClass.modifiedRotateCurrent() < 50 && RotateClass.modifiedRotateCurrent() > -50 && ExtendClass.extendModifiedEncoder > 350 && ExtendClass.extendModifiedEncoder < 450){
                        robot.TP_M.setPower(VPivotClass.NEWVPivot(450, SPEEDSET, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM,DNDM,MINSPEED));
                        vPivotSetPoint = 450;
                    }
                }else{
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(550, SPEEDSET, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM,DNDM,MINSPEED));
                    vPivotSetPoint = 550;
                }

            }else if(gamepad2.dpad_right){
                robot.TP_M.setPower(VPivotClass.NEWVPivot(1500, SPEEDSET, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM,DNDM,MINSPEED));
                vPivotSetPoint = 1500;
                if(VPivotClass.encoderWithOffset > 1450){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(1500, .7, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                    rotateSet = 1500;
                    robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(500, .7, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                    extendSetPoint = 500;
                }

            }else if(gamepad2.dpad_up){
                robot.TP_M.setPower(VPivotClass.NEWVPivot(1000, SPEEDSET, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM,DNDM,MINSPEED));
                vPivotSetPoint = 1000;
                if(VPivotClass.encoderWithOffset > 1450){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0, .7, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                    rotateSet = 0;
                    robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(50, .7, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                    extendSetPoint = 50;
                }


            }else if(gamepad2.dpad_left){
                robot.TP_M.setPower(VPivotClass.NEWVPivot(800, SPEEDSET, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM,DNDM,MINSPEED));
                vPivotSetPoint = 800;
                if(VPivotClass.encoderWithOffset > 750){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(-1500, .7, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                    rotateSet = -1500;
                    robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(50, .7, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                    extendSetPoint = 50;
                }

            } else{
                vPivotSetPoint = vPivotSetPoint - (gamepad2.left_stick_y * 30);
                robot.TP_M.setPower(VPivotClass.NEWVPivot(vPivotSetPoint, SPEEDSET, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM,DNDM,MINSPEED));

                if(gamepad2.right_trigger > .05){
                    rotateSet = rotateSet + (gamepad2.right_trigger * 30);
                }else if(gamepad2.left_trigger > .05){
                    rotateSet = rotateSet - (gamepad2.left_trigger * 30);
                }
                robot.TR_M.setPower(RotateClass.RotateAutoMethod(rotateSet, 1, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));

                extendSetPoint = extendSetPoint - (gamepad2.right_stick_y * 30);
                robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(extendSetPoint, 1, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));

            }*/




            if(gamepad2.dpad_down){

                rotateSet = 0;
                ROTATESPEED = 1;

                extendSetPoint = 400;
                EXTENDSPEED = 1;
                if( 50 > Math.abs(RotateClass.modifiedRotateCurrent()) && ExtendClass.extendModifiedEncoder < 450 && ExtendClass.extendModifiedEncoder > 350){
                    vPivotSetPoint = 400;
                    VPIVOTSPEED = 16;
                }else{
                    vPivotSetPoint = 750;
                    VPIVOTSPEED = 16;
                }




            }else if(gamepad2.dpad_right){
                vPivotSetPoint = 1700;
                VPIVOTSPEED = 20;
                if(VPivotClass.encoderWithOffset > 1450){
                    ROTATESPEED = 1;
                    rotateSet = 1500;
                    EXTENDSPEED = 1;
                    extendSetPoint = 500;
                }

            }else if(gamepad2.dpad_up){
                vPivotSetPoint = 1200;
                VPIVOTSPEED = 20;
                if(VPivotClass.encoderWithOffset > 1450){
                    ROTATESPEED = 1;
                    rotateSet = 0;
                    EXTENDSPEED = 1;
                    extendSetPoint = 50;
                }


            }else if(gamepad2.dpad_left){
                vPivotSetPoint = 800;
                VPIVOTSPEED = 16;
                if(VPivotClass.encoderWithOffset > 750){
                    ROTATESPEED = 16;
                    rotateSet = -1500;
                    EXTENDSPEED = 1;
                    extendSetPoint = 50;
                }

            } else{
                vPivotSetPoint = vPivotSetPoint - (gamepad2.left_stick_y * 30);
                VPIVOTSPEED = 16;

                if(gamepad2.right_trigger > .05){
                    rotateSet = rotateSet + (gamepad2.right_trigger * 30);
                }else if(gamepad2.left_trigger > .05){
                    rotateSet = rotateSet - (gamepad2.left_trigger * 30);
                }
                ROTATESPEED = 1;

                extendSetPoint = extendSetPoint - (gamepad2.right_stick_y * 30);
                EXTENDSPEED = 1;

            }
            robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(extendSetPoint, EXTENDSPEED, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
            robot.TR_M.setPower(RotateClass.RotateAutoMethod(rotateSet, ROTATESPEED, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
           // robot.TP_M.setPower(VPivotClass.NEWVPivot(vPivotSetPoint, VPIVOTSPEED, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM,DNDM,MINSPEED));





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
                robot.RI_S.setPower(-.6);
                robot.LI_S.setPower(.6);
            }else if(gamepad1.b || gamepad2.b){
                robot.RI_S.setPower(.6);
                robot.LI_S.setPower(-.6);
            }else{
                robot.RI_S.setPower(0);
                robot.LI_S.setPower(0);
            }

            Telemetry();
        }

    }
    public void Telemetry(){
        telemetry.addData("speedSet", VPivotClass.speedSetPoint);
       // telemetry.addData("POT", robot.TP_P.getVoltage());
        telemetry.addData("motor power", VPivotClass.FinalMotorPower);
        telemetry.addData("setpt", VPivotClass.vPivotSet);
        telemetry.addData("DegreesTraveled", VPivotClass.DegreesTravelReturn());
        telemetry.addData("deltaEncoder", VPivotClass.deltaPivotEncoder);
        telemetry.addData("EncoderWithOffset", VPivotClass.encoderWithOffset);
        telemetry.addData("speed", VPivotClass.SpeedReturn());
        telemetry.addData("degrees Traveled", VPivotClass.DegreesTravelReturn());
        telemetry.addData("pivot encoder", robot.TP_M.getCurrentPosition());

        dashboardTelemetry.addData("motor power", VPivotClass.FinalMotorPower);
        dashboardTelemetry.addData("speed", VPivotClass.SpeedReturn());
        dashboardTelemetry.addData("DegreesTraveled", VPivotClass.DegreesTravelReturn());
        dashboardTelemetry.addData("deltaEncoder", VPivotClass.deltaPivotEncoder);
        dashboardTelemetry.addData("RATIOEncoderWithOffset", VPivotClass.encoderWithOffset/1000);
        dashboardTelemetry.addData("EncoderWithOffset", VPivotClass.encoderWithOffset);
        dashboardTelemetry.addData("Pivot Encoder", robot.TP_M.getCurrentPosition());
       // dashboardTelemetry.addData("POT", robot.TP_P.getVoltage());
        dashboardTelemetry.update();
        telemetry.update();
    }

}
