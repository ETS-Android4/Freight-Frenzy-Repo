package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.TestHub.TurretTesting;
import org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass;
import org.firstinspires.ftc.teamcode.TurretClasses.RotateClass;
import org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass;

@Config
@TeleOp
public class NewTurretTeleOp extends LinearOpMode{

    public static double UPARMPM = .009;
    public static double UPARMDM = .008;
    public static double DNPM = .008;
    public static double DNDM = .005;
    public  static double SPEEDSET = 16;
    public static double MINSPEED = .2;
    public static double SETPOINT = 1500;

    public double vPivotSetPoint = 1500, extendSetPoint = 0, rotateSet = 0;
    public double x, y, z;


    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    VPivotClass VPivotClass = new VPivotClass();
    ExtendClass ExtendClass = new ExtendClass();
    RotateClass RotateClass = new RotateClass();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    public void runOpMode(){
        robot.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()){

            vPivotSetPoint = vPivotSetPoint - (gamepad2.left_stick_y * 30);
            robot.TP_M.setPower(VPivotClass.NEWVPivot(vPivotSetPoint, SPEEDSET, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM,DNDM,MINSPEED));

            extendSetPoint = extendSetPoint - (gamepad2.right_stick_y * 30);
            robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(extendSetPoint, 1, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));

            if(gamepad2.right_trigger > .05){
                rotateSet = rotateSet + (gamepad2.right_trigger * 20);
            }else if(gamepad2.left_trigger > .05){
                rotateSet = rotateSet - (gamepad2.left_trigger * 20);
            }

            robot.TR_M.setPower(RotateClass.RotateAutoMethod(rotateSet, 1, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));

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

            Telemetry();
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

        dashboardTelemetry.addData("motor power", VPivotClass.FinalMotorPower);
        dashboardTelemetry.addData("speed", VPivotClass.SpeedReturn());
        dashboardTelemetry.addData("DegreesTraveled", VPivotClass.DegreesTravelReturn());
        dashboardTelemetry.addData("deltaEncoder", VPivotClass.deltaPivotEncoder);
        dashboardTelemetry.addData("RATIOEncoderWithOffset", VPivotClass.encoderWithOffset/1000);
        dashboardTelemetry.addData("EncoderWithOffset", VPivotClass.encoderWithOffset);
        dashboardTelemetry.addData("Pivot Encoder", robot.TP_M.getCurrentPosition());
        dashboardTelemetry.addData("POT", robot.TP_P.getVoltage());
        dashboardTelemetry.update();
        telemetry.update();
    }

}
