package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.TestHub.Smoothing;
import org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass;
import org.firstinspires.ftc.teamcode.TurretClasses.RotateClass;
import org.firstinspires.ftc.teamcode.TurretClasses.TurretCombined;
import org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass;


@Config
@TeleOp
public class NewTurretBlue extends LinearOpMode{



    public double x, y, z, initPOsitionOrder = 1, xOriginal;

    double teleOpExtendSet = 200, teleOpRotateSet = 0, teleOpVPivotSet = 1000;
    double teleOpExtendSpeedSet = 25, teleOpRotateSpeedSet = 1500, teleOpVPivotSpeedSet = 16;


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



        waitForStart();

        while (opModeIsActive()){







            x = Smoothing.SmoothDriveX(-Math.copySign(gamepad1.left_stick_x, gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x));
            y = Smoothing.SmoothDriveY( -(Math.copySign(gamepad1.left_stick_y, gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y)));
            z = Smoothing.SmoothDriveZ(  Math.copySign(gamepad1.right_stick_x, gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x));




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


            if(gamepad2.left_bumper){
                if(gamepad2.b){
                    robot.TC_M.setPower(0);
                }else if(gamepad2.a){
                    robot.TC_M.setPower(1);
                }else if(gamepad2.x){
                    robot.TC_M.setPower(-1);
                }else{
                    robot.TC_M.setPower(robot.TC_M.getPower() + (gamepad2.right_stick_y * .01));
                }
            }else{
                if(gamepad1.a || gamepad2.a){
                    robot.RI_S.setPower(-.5);
                    robot.LI_S.setPower(.5);
                }else if(gamepad1.b || gamepad2.b){
                    robot.RI_S.setPower(.5);
                    robot.LI_S.setPower(-.5);
                }else{
                    robot.RI_S.setPower(0);
                    robot.LI_S.setPower(0);
                }
                if(gamepad2.dpad_right) {
                    teleOpVPivotSet = 1600;
                    if (CombinedTurret.vPivotModifiedEncoder > 1000) {
                        teleOpRotateSet = 1300;
                        teleOpExtendSet = 500;
                    }
                }else if(gamepad2.dpad_down) {
                    teleOpVPivotSet = 1000;
                    if (CombinedTurret.vPivotModifiedEncoder > 800) {
                        teleOpRotateSet = 0;
                        teleOpExtendSet = 300;
                    } else if (Math.abs(CombinedTurret.rotateModifiedEncoder) < 50 && CombinedTurret.extendModifiedEncoder < 250) {
                        teleOpVPivotSet = 300;
                    }
                }else if(gamepad2.dpad_left){
                    teleOpVPivotSet = 1000;
                    if (CombinedTurret.vPivotModifiedEncoder > 800) {
                        teleOpRotateSet = -1000;
                        teleOpExtendSet = 200;
                    }

                }else{
                    teleOpExtendSet = teleOpExtendSet - (gamepad2.right_stick_y * 30);
                    if(gamepad2.right_trigger > .05){
                        teleOpRotateSet = teleOpRotateSet + (gamepad2.right_trigger * 30);
                    }else if(gamepad2.left_trigger > .05){
                        teleOpRotateSet = teleOpRotateSet - (gamepad2.left_trigger * 30);
                    }

                    teleOpVPivotSet = teleOpVPivotSet + (gamepad2.left_stick_y * -20);

                }
            }
            if(teleOpVPivotSet > 1800){
                teleOpVPivotSet = 1800;
            }else if(teleOpVPivotSet < 100){
                teleOpVPivotSet = 100;
            }

            if(teleOpExtendSet > 1200){
                teleOpExtendSet = 1200;
            }if(teleOpExtendSet < 0){
                teleOpExtendSet = 0;
            }

            if(teleOpRotateSet > 5000){
                teleOpRotateSet = 5000;
            }else if(teleOpRotateSet < -5000){
                teleOpRotateSet = -5000;
            }

            CombinedTurret.TurretCombinedMethod(teleOpExtendSet,teleOpExtendSpeedSet,teleOpRotateSet,teleOpRotateSpeedSet, teleOpVPivotSet,teleOpVPivotSpeedSet, robot.TE_M.getCurrentPosition(), robot.TE_G.getState(), robot.TR_M.getCurrentPosition(), robot.TR_G.getState(), robot.TP_M.getCurrentPosition(), robot.TP_G.getState());

            robot.TE_M.setPower(CombinedTurret.extendFinalMotorPower);
            robot.TR_M.setPower(CombinedTurret.rotateFinalMotorPower);
            robot.TP_M.setPower(CombinedTurret.vPivotFinalMotorPower);

            Telemetry();
        }

    }
    public void Telemetry(){

        dashboardTelemetry.addData("vPivotMotor Power", CombinedTurret.vPivotFinalMotorPower);
        dashboardTelemetry.addData("vPivot Modified Encoder", CombinedTurret.vPivotModifiedEncoder);

        dashboardTelemetry.addData("extend Motor Power", CombinedTurret.extendFinalMotorPower);
        dashboardTelemetry.addData("extend speed", CombinedTurret.extendSpeed);
        dashboardTelemetry.addData("extend Modified Encoder", CombinedTurret.extendModifiedEncoder);
        dashboardTelemetry.addData("extend Set", CombinedTurret.extendSet);
        dashboardTelemetry.addData("turret Homing trigger", CombinedTurret.turretHomingTrigger);

        telemetry.addData("intake Dist", robot.I_DS.getDistance(DistanceUnit.INCH));

        dashboardTelemetry.update();
        telemetry.update();
    }

}
