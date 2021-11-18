package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Test;
//@TeleOp
public class ColorSensorTest extends LinearOpMode {

    TestHubHardware robot = new TestHubHardware();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            NormalizedRGBA TurretHome = robot.TurretHome.getNormalizedColors();

            telemetry.addData("color1", TurretHome.alpha);
            telemetry.addData("color1", TurretHome.red);
            telemetry.addData("color1", TurretHome.blue);
            telemetry.addData("color1", TurretHome.green);

            if(robot.MagneticRotate.getState() == true){
                telemetry.addData("magispressed00000000000000000000",0);
            }else{
                telemetry.addData("magisnotpressed",0);
            }


            telemetry.update();

        }
    }

}

