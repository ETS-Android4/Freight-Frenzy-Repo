package org.firstinspires.ftc.teamcode.TurretClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;

//@Autonomous
@Config
//@Autonomous
public class NEWHomingProgram extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static double UPARMPM = .009;
    public static double UPARMDM = .008;
    public static double DNPM = .008;
    public static double DNDM = .005;
    public  static double SPEEDSET = 16;
    public static double MINSPEED = .2;
    public static double SETPOINT = 1500;
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    VPivotClass VPivotClass = new VPivotClass();
    RotateClass RotateClass = new RotateClass();
    ExtendClass ExtendClass = new ExtendClass();
    double initPOsitionOrder = 1;

    public void runOpMode(){
        robot.init(hardwareMap);
     /*   while (!opModeIsActive()){
            if(RotateClass.isHomedRotateReturn() == false){
                if(VPivotClass.has1stloop == true){
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(2000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));
                    if(VPivotClass.encoderWithOffset > 1900 && VPivotClass.encoderWithOffset < 2100){
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
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(800,.8,robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                    if(RotateClass.modifiedRotateCurrent() > 750 && RotateClass.modifiedRotateCurrent() < 850){
                        initPOsitionOrder = 2;
                    }
                }else if(initPOsitionOrder == 2){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(800,.8,robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(500, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));
                    if(VPivotClass.encoderWithOffset < 550 && VPivotClass.encoderWithOffset > 450){
                        initPOsitionOrder = 3;
                    }
                }else if(initPOsitionOrder == 3){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(650,.4,robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(500, 5, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));

                }



            }
            telemetry.addData("Rotate homed boolean", RotateClass.isHomedRotateReturn());
            telemetry.addData("initPosition order", initPOsitionOrder);
            telemetry.addData("Vpiovot PT", robot.TP_P.getVoltage());
            telemetry.addData("rotate modified", RotateClass.modifiedRotateCurrent());
            telemetry.update();
        }*/

        waitForStart();
        while(opModeIsActive()){
/*
            robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15,.5, robot.TP_P.getVoltage()));
            if(robot.TP_P.getVoltage() > 1.1 && robot.TP_P.getVoltage() < 1.25){
                robot.TE_M.setPower(ExtendClass.ExtendHoming(robot.TE_G.getState(), robot.TE_M.getCurrentPosition()));
                if(ExtendClass.isHomedExtendReturn() == true){
                    robot.TR_M.setPower(RotateClass.RotateHoming(robot.TR_G.getState(), robot.TR_M.getCurrentPosition()));
                }
            } */

            telemetry.addData("mag sensor", robot.TE_G.getState());
            telemetry.addData("homiing next set", ExtendClass.HomingnextSetReturn());
          //  telemetry.addData("POT reading", robot.TP_P.getVoltage());
            telemetry.addData("extend mp", robot.TE_M.getPower());
            telemetry.addData("EXtend is homed", ExtendClass.isHomedExtendReturn());
            telemetry.addData("rotate is homed", RotateClass.isHomedRotateReturn());

            telemetry.update();

        }
    }

}
