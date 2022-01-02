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

    public static double UPARMPM = .004;
    public static double UPARMDM = .006;
    public static double DNPM = .0005;
    public static double DNDM = .0003;
    public  static double SPEEDSET = 16;
    public static double MINSPEED = .2;

    public double vPivotSetPoint = 1500, extendSetPoint = 0;


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

            vPivotSetPoint = vPivotSetPoint + gamepad2.left_stick_y;
            robot.TP_M.setPower(VPivotClass.NEWVPivot(vPivotSetPoint, 35, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM,DNDM,MINSPEED));

            extendSetPoint = extendSetPoint + gamepad2.right_stick_y;
            ExtendClass.ExtendAutoMethod(extendSetPoint, 1, ExtendClass.extendModifiedEncoder, robot.TE_G.getState());

            RotateClass.RotateMethod(gamepad2.right_trigger, gamepad2.left_trigger, RotateClass.modifiedRotateCurrent(), robot.TR_G.getState());

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
