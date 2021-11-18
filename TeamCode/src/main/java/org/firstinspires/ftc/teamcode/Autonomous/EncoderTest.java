package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.Odometry;
import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;

//@TeleOp
public class EncoderTest extends LinearOpMode {
    Odometry OdoClass = new Odometry();
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
           OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
           telemetry.addData("X", OdoClass.odoXReturn());
           telemetry.addData("Y", OdoClass.odoYReturn());
           telemetry.addData("Theta", OdoClass.thetaInDegreesReturn());
            telemetry.addData("E1", robot.LF_M.getCurrentPosition());
            telemetry.addData("E2", robot.LB_M.getCurrentPosition());
            telemetry.addData("E3", robot.RF_M.getCurrentPosition());
            telemetry.update();
        }
    }
}
