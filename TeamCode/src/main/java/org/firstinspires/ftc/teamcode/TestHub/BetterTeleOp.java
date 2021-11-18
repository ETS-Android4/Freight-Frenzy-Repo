package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RotateClass;
import org.firstinspires.ftc.teamcode.TestHub.ExtendClass;
import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.TestHub.VPivotClass;

//@TeleOp
public class BetterTeleOp extends LinearOpMode {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    org.firstinspires.ftc.teamcode.TestHub.ExtendClass ExtendClass = new ExtendClass();
    org.firstinspires.ftc.teamcode.TestHub.VPivotClass VPivotClass = new VPivotClass();
    org.firstinspires.ftc.teamcode.RotateClass RotateClass = new RotateClass();
    double x, y, z;

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            //getting exponential joystick control for the drivetrain
            x = Math.copySign(gamepad1.left_stick_x, gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x);
            y = Math.copySign(gamepad1.left_stick_y, gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y);
            z = Math.copySign(gamepad1.right_stick_x, gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

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


            robot.TE_M.setPower(ExtendClass.ExtendMethod((-gamepad2.left_stick_y), robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
            robot.TP_M.setPower(VPivotClass.VPivotMethod(gamepad2.right_stick_y, robot.TP_P.getVoltage()));
            robot.TR_M.setPower(RotateClass.RotateMethod(gamepad2.right_trigger, gamepad2.left_trigger, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));


            telemetry.update();
        }
    }
}
