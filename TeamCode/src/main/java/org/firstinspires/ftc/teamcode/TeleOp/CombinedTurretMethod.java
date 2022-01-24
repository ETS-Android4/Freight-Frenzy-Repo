package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.TestHub.Smoothing;
import org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass;
import org.firstinspires.ftc.teamcode.TurretClasses.RotateClass;
import org.firstinspires.ftc.teamcode.TurretClasses.TurretCombined;
import org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass;


@Config
@TeleOp
public class CombinedTurretMethod extends LinearOpMode{

    public static double UPARMPM = .01;
    public static double UPARMDM = .006;
    public static double DNPM = .005;
    public static double DNDM = .003;
    public static double EXTENDSPEED = 20;
    public static double EXTENDP = .015;
    public static double EXTENDD = 0.012;
    public static double ROTATEP = 0.0003;
    public static double ROTATED = 0.0003;
    public static double EXTENDSET = 200;
    public static double VPIVOTSPEEDSET = 16;
    public static double VPIVOTSETPOINT = 1500;
    public static double ROTATESPEED = 500;
    public static double ROTATESET = 10;


    public double vPivotSetPoint = 1500, extendSetPoint = 0;
    public double x, y, z, initPOsitionOrder = 1, xOriginal;


    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    TurretCombined CombinedTurret = new TurretCombined();
    VPivotClass VPivotClass = new VPivotClass();
    ExtendClass ExtendClass = new ExtendClass();
    RotateClass RotateClass = new RotateClass();
    Smoothing Smoothing = new Smoothing();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();



    @Override
    public void runOpMode(){
        robot.init(hardwareMap);


        while (!opModeIsActive()){
            if(RotateClass.isHomedRotateReturn() == false){
                if(VPivotClass.has1stloop == true){
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(2000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, .2));
                }else{
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(3000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, .2));
                }

            }
        }





        waitForStart();

        while (opModeIsActive()){

            CombinedTurret.vPivotUpPm = UPARMPM;
            CombinedTurret.vPivotUpDM = UPARMDM;
            CombinedTurret.vPivotDnPM = DNPM;
            CombinedTurret.vPivotDnDM = DNDM;

            CombinedTurret.extendPM = EXTENDP;
            CombinedTurret.extendDM = EXTENDD;

            CombinedTurret.rotatePM = ROTATEP;
            CombinedTurret.extendDM = ROTATED;

            CombinedTurret.TurretCombinedMethod(EXTENDSET,EXTENDSPEED,ROTATESET,ROTATESPEED, VPIVOTSETPOINT,VPIVOTSPEEDSET, robot.TE_M.getCurrentPosition(), robot.TE_G.getState(), robot.TR_M.getCurrentPosition(), robot.TR_G.getState(), robot.TP_M.getCurrentPosition(), robot.TP_G.getState());

            robot.TE_M.setPower(CombinedTurret.extendFinalMotorPower);
            robot.TR_M.setPower(CombinedTurret.rotateFinalMotorPower);
            robot.TP_M.setPower(CombinedTurret.vPivotFinalMotorPower);

            x = -(Math.copySign(gamepad1.left_stick_x, gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x));
            xOriginal = -(Math.copySign(gamepad1.left_stick_y, gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y));
            z = Math.copySign(gamepad1.right_stick_x, gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

            y = Smoothing.SmoothTest(xOriginal);



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

            Telemetry();
        }

    }
    public void Telemetry(){

        dashboardTelemetry.addData("vPivotMotor Power", CombinedTurret.vPivotFinalMotorPower);
        dashboardTelemetry.addData("vPivot Modified Encoder", CombinedTurret.vPivotModifiedEncoder);

        dashboardTelemetry.addData("extend Motor Power", CombinedTurret.extendFinalMotorPower);
        dashboardTelemetry.addData("extend speed", CombinedTurret.extendSpeed);

        dashboardTelemetry.update();
        telemetry.update();
    }

}
