package org.firstinspires.ftc.teamcode.TestHub;

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
    public static double ENCODER_EQUATION_MULT = .00045;
    public static double POT_SUB_AMT = 2.092;
    TurretTesting robot = new TurretTesting();
    VPivotClass VPivotClass = new VPivotClass();


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        waitForStart();
        while (opModeIsActive()) {

            VPivotClass.NEWVPivot(1,10, robot.TP_P.getVoltage(), robot.TP_M.getCurrentPosition(), getRuntime(), 16);

            telemetry.addData("POT", robot.TP_P.getVoltage());
            telemetry.addData("neg degrees", (robot.TP_P.getVoltage() -2.38) / -.0105);


            telemetry.addData("DegreesTraveled", VPivotClass.DegreesTravelReturn());
            telemetry.addData("deltaEncoder", VPivotClass.deltaPivotEncoder);
            telemetry.addData("EncoderWithOffset", VPivotClass.encoderWithOffset);
            telemetry.addData("speed", VPivotClass.SpeedReturn());
            telemetry.addData("degrees Traveled", VPivotClass.DegreesTravelReturn());
            telemetry.addData("last POT", VPivotClass.LastPOTreadingReturn());
            telemetry.addData("time", getRuntime());
            telemetry.addData("deltaPivot", VPivotClass.deltaPivot);
            telemetry.addData("pivot encoder", robot.TP_M.getCurrentPosition());
            telemetry.addData("encoder equation", (robot.TP_P.getVoltage() - POT_SUB_AMT)/ENCODER_EQUATION_MULT);

            dashboardTelemetry.addData("speed", VPivotClass.SpeedReturn());

            dashboardTelemetry.addData("DegreesTraveled", VPivotClass.DegreesTravelReturn());
            dashboardTelemetry.addData("deltaEncoder", VPivotClass.deltaPivotEncoder);
            dashboardTelemetry.addData("EncoderWithOffset", VPivotClass.encoderWithOffset);
            dashboardTelemetry.addData("Pivot Encoder", robot.TP_M.getCurrentPosition());
            dashboardTelemetry.addData("time", getRuntime());
            dashboardTelemetry.addData("POT", -robot.TP_P.getVoltage());
            dashboardTelemetry.addData("last POT", VPivotClass.LastPOTreadingReturn());
            dashboardTelemetry.addData("deltaPivot", VPivotClass.deltaPivot);

            dashboardTelemetry.addData("encoder equation", (robot.TP_P.getVoltage() - POT_SUB_AMT)/ENCODER_EQUATION_MULT);
            dashboardTelemetry.update();



            telemetry.update();


        }
    }

}

