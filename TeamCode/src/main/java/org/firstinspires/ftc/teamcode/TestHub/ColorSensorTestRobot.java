package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
public class ColorSensorTestRobot extends LinearOpMode {

    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {


            telemetry.addData("LF_C Alpha", robot.LF_C.alpha());


            telemetry.addData("LB_C Alpha", robot.LB_C.alpha());

            telemetry.addData("RF_C Alpha", robot.RF_C.alpha());

            telemetry.addData("RB_C Alpha", robot.RB_C.alpha());





            telemetry.update();

        }
    }

}

