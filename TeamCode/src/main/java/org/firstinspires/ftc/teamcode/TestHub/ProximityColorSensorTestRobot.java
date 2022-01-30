package org.firstinspires.ftc.teamcode.TestHub;

import android.sax.TextElementListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ProximityColorSensorTestRobot extends LinearOpMode {

    TestHubHardware robot = new TestHubHardware();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("IN", robot.PC.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }
    }

}

