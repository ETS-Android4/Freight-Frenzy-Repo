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
public class NewTurretProgrammingTeleOP extends LinearOpMode{

    public static double UPARMPM = .015;
    public static double UPARMDM = .009;
    public static double DNPM = .004;
    public static double DNDM = .012;
    public static double EXTENDSPEED = 5;
    public static double EXTENDP = .02;
    public static double EXTENDD = 0.01;
    public static double EXTENDI = 0;
    public static double ROTATEP = .02;
    public static double ROTATED = 0.01;
    public static double ROTATEI = 0;
    public static double EXTENDSET = 200;
    public static double VPIVOTSPEEDSET = 16;
    public static double VPIVOTSETPOINT = 1500;
    public static double ROTATESPEED = 1;
    public static double ROTATESET = 10;


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
/*

        while (!opModeIsActive()){
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
        }



*/

        waitForStart();

        while (opModeIsActive()){

            VPivotClass.UPSpeedPM = UPARMPM;
            VPivotClass.UPSpeedDM = UPARMDM;
            VPivotClass.DNSpeedPM = DNPM;
            VPivotClass.DNSpeedDM = DNDM;

            ExtendClass.extendP = EXTENDP;
            ExtendClass.extendD = EXTENDD;
            ExtendClass.extendI = EXTENDI;

            RotateClass.rotateP = ROTATEP;
            RotateClass.rotateD = ROTATED;
            RotateClass.rotateI = ROTATEI;

            robot.TE_M.setPower(ExtendClass.ExtendSpeedMethod(EXTENDSET, EXTENDSPEED, robot.TE_G.getState(), robot.TE_M.getCurrentPosition()));
            robot.TR_M.setPower(RotateClass.RotateSpeedMethod(ROTATESET, ROTATESPEED, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
            VPivotClass.VPivot(VPIVOTSETPOINT, VPIVOTSPEEDSET, robot.TP_M.getCurrentPosition(), robot.TP_G.getState());


            x = -(Math.copySign(gamepad1.left_stick_x, gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x));
            y = -(Math.copySign(gamepad1.left_stick_y, gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y));
            z = Math.copySign(gamepad1.right_stick_x, gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);
/*
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

           */ Telemetry();
        }

    }
    public void Telemetry(){
        telemetry.addData("speedSet", VPivotClass.speedSetPoint);
        telemetry.addData("POT", robot.TP_P.getVoltage());
        telemetry.addData("motor power", VPivotClass.FinalMotorPower);
        telemetry.addData("setpt", VPivotClass.vPivotSet);
        telemetry.addData("DegreesTraveled", VPivotClass.DegreesTravelReturn());
        telemetry.addData("deltaEncoder", VPivotClass.deltaPivotEncoder);
        telemetry.addData("EncoderWithOffset", VPivotClass.encoderWithOffset);
        telemetry.addData("speed", VPivotClass.SpeedReturn());
        telemetry.addData("degrees Traveled", VPivotClass.DegreesTravelReturn());
        telemetry.addData("pivot encoder", robot.TP_M.getCurrentPosition());

        dashboardTelemetry.addData("Extend Encoder", robot.TE_M.getCurrentPosition());
        dashboardTelemetry.addData("Extend Modified Encoder", ExtendClass.extendModifiedEncoder);
        dashboardTelemetry.addData("Extend Motor Power", ExtendClass.ExtendMotorPower);
        dashboardTelemetry.addData("Extend Speed", ExtendClass.extendCurrentSpeed);
        dashboardTelemetry.addData("Extend Intregal Sum", ExtendClass.extendIntegralsum);
        dashboardTelemetry.addData("Extend Speed SET", ExtendClass.extendSpeedSet);
        /*
        dashboardTelemetry.addData("VPivot Modified Encoder",VPivotClass.encoderWithOffset);
        dashboardTelemetry.addData("VPivot Speed", VPivotClass.speed);
        dashboardTelemetry.addData("VPivot Motor Power", VPivotClass.FinalMotorPower);
        */
        dashboardTelemetry.addData("Rotate Modfiied Encoder", RotateClass.modifiedCurrentPos);
        dashboardTelemetry.addData("Rotate speed", RotateClass.currentSpeed);
        dashboardTelemetry.addData("Rotate motorPower", RotateClass.rotateMotorPower);

        dashboardTelemetry.addData("nanotimerin seconds", System.nanoTime()/1000000000);
        dashboardTelemetry.addData("elapsed Time", robot.TimerCustom());
        dashboardTelemetry.update();
        telemetry.update();
    }

}
