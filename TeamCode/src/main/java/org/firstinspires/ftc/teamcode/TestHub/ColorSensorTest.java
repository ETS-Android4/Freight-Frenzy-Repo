package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Test;
@TeleOp
public class ColorSensorTest extends LinearOpMode {

    TestHubHardware robot = new TestHubHardware();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            NormalizedRGBA ColorSensor = robot.ColorSensor.getNormalizedColors();

            telemetry.addData("color1", ColorSensor.alpha);
            telemetry.addData("color1", ColorSensor.red);
            telemetry.addData("color1", ColorSensor.blue);
            telemetry.addData("color1", ColorSensor.green);




            telemetry.update();

        }
    }

}

