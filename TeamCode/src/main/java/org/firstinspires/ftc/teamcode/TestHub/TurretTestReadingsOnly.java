package org.firstinspires.ftc.teamcode.TestHub;

//import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass;

@Config
@TeleOp
public class TurretTestReadingsOnly extends LinearOpMode {
    public static double UPARMPM = .015;
    public static double UPARMDM = .009;
    public static double DNPM = .004;
    public static double DNDM = .012;
    public  static double SPEEDSET = 16;
    public static double MINSPEED = .2;
    public static double SETPOINT = 1500;
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    VPivotClass VPivotClass = new VPivotClass();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();
        while (opModeIsActive()) {


            robot.TP_M.setPower( VPivotClass.NEWVPivot(SETPOINT ,SPEEDSET , robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(), robot.TP_G.getState() ,getRuntime(), 16, UPARMPM, UPARMDM, DNPM, DNDM, .2));

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
            dashboardTelemetry.addData("encoder", robot.TP_M.getCurrentPosition());
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

}

