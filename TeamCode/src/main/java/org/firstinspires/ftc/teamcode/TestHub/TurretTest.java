package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp
public class TurretTest extends LinearOpMode {
    TestHubHardware robot = new TestHubHardware();

    double extendMin = 2, extendMax = 1300, extendSet = extendMin;
    double extendDifference = 0, extendMultiplied = 0, extendP = -.02, extendD = 0;


    double vPivotMin = 0.8, vPivotMax = 3, vPivotSet = 1.15;
    double vPivotDifference = 0, vPivotMultiplied = 0, vPivotP = 3, vPivotD = 0;

    double rotateMin = -5000, rotateMax = 5000, rotateSet = 0;
    double rotateDifference = 0, rotateMultiplied = 0, rotateP = -.01, rotateD = 0;

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            //Linear Slides Extend

            extendSet = extendSet + (15 * (gamepad1.left_stick_x));
            vPivotSet = vPivotSet + (.02 * gamepad1.right_stick_y);
            rotateSet = rotateSet + (16 * gamepad1.right_stick_x);
            //Setpoint limits
            if(extendSet < extendMin){
                extendSet = (extendMin + 2);
            }else  if( extendSet > extendMax){
                extendSet = (extendMax - 2);
            }

            extendDifference = robot.Motor1.getCurrentPosition() - extendSet;
            extendMultiplied = extendDifference * extendP;

            //VPivot Limits
            if(vPivotSet < vPivotMin){
                vPivotSet = vPivotMin;
            }else if(vPivotSet > vPivotMax){
                vPivotSet = vPivotMax;
            }

            vPivotDifference = robot.PivotPT.getVoltage() - vPivotSet;
            vPivotMultiplied = vPivotDifference * vPivotP;


            //Hpivot Limits
            if(rotateSet < rotateMin){
                rotateSet = rotateMin;
            }else if(rotateSet > rotateMax){
                rotateSet = rotateMax;
            }

            rotateDifference = robot.Motor3.getCurrentPosition() - rotateSet;
            rotateMultiplied = rotateDifference * rotateP;

            robot.Motor1.setPower(extendMultiplied);
            robot.Motor2.setPower(vPivotMultiplied);
            robot.Motor3.setPower(rotateMultiplied);


            telemetry.addData("vPivotSet", vPivotSet);
            telemetry.addData("rotateSet", rotateSet);
            telemetry.addData("extendSet", extendSet);
            telemetry.addData("motor1", robot.Motor1.getPower());
            telemetry.addData("extendcurrent", robot.Motor1.getCurrentPosition());
            telemetry.addData("rotate power", rotateMultiplied);
            telemetry.addData("Vpivot power", vPivotMultiplied);
            telemetry.addData("extend power", extendMultiplied);
            telemetry.addData("pot", robot.PivotPT.getVoltage());
            telemetry.update();
        }
    }
}
