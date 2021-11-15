package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RotateClass;

@TeleOp
public class HomingProgram extends LinearOpMode {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    ExtendClass ExtendClass = new ExtendClass();
    VPivotClass VPivotClass = new VPivotClass();
    RotateClass RotateClass = new RotateClass();
    double initPOsitionOrder = 1;

    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()){
            if(RotateClass.isHomedRotateReturn() == false){
                robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15,.5, robot.TP_P.getVoltage()));
                if(robot.TP_P.getVoltage() > 1.1 && robot.TP_P.getVoltage() < 1.25){
                    robot.TE_M.setPower(ExtendClass.ExtendHoming(robot.TE_G.getState(), robot.TE_M.getCurrentPosition()));
                    if(ExtendClass.isHomedExtendReturn() == true){
                        robot.TR_M.setPower(RotateClass.RotateHoming(robot.TR_G.getState(), robot.TR_M.getCurrentPosition()));
                    }
                }
            }else{
                telemetry.addData("homed",0);
                robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(10,.8,robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                if(initPOsitionOrder == 1){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(500,.8,robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                    if(RotateClass.modifiedRotateCurrent() > 450 && RotateClass.modifiedRotateCurrent() < 550){
                        initPOsitionOrder = 2;
                    }
                }else if(initPOsitionOrder == 2){
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.8,1, robot.TP_P.getVoltage()));
                    if(robot.TP_P.getVoltage() < 2 && robot.TP_P.getVoltage() > 1.6){
                        initPOsitionOrder = 3;
                    }
                }else if(initPOsitionOrder == 3){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(200,.4,robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.65,1, robot.TP_P.getVoltage()));

                }



            }
            telemetry.addData("Rotate homed boolean", RotateClass.isHomedRotateReturn());
            telemetry.addData("initPosition order", initPOsitionOrder);
            telemetry.addData("Vpiovot PT", robot.TP_P.getVoltage());
            telemetry.addData("rotate modified", RotateClass.modifiedRotateCurrent());
            telemetry.update();
        }

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
            telemetry.addData("POT reading", robot.TP_P.getVoltage());
            telemetry.addData("extend mp", robot.TE_M.getPower());
            telemetry.addData("EXtend is homed", ExtendClass.isHomedExtendReturn());
            telemetry.addData("rotate is homed", RotateClass.isHomedRotateReturn());

            telemetry.update();

        }
    }

}
