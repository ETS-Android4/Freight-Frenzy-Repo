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

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){

            VPivotClass.VPivotAutoMethod(1.15,.5, robot.TP_P.getVoltage());
            /*if(robot.TP_P.getVoltage() > 1.1 && robot.TP_P.getVoltage() < 1.25){
                ExtendClass.ExtendHoming(robot.TE_G.getState(), robot.TE_M.getCurrentPosition());
                if(ExtendClass.isHomedExtendReturn() == true){
                    RotateClass.RotateHoming(robot.TR_G.getState(), robot.TR_M.getCurrentPosition());
                }
            }*/

            telemetry.addData("POT reading", robot.TP_P.getVoltage());
            telemetry.addData("EXtend is homed", ExtendClass.isHomedExtendReturn());
            telemetry.addData("rotate is homed", RotateClass.isHomedRotateReturn());

            telemetry.update();

        }
    }

}
