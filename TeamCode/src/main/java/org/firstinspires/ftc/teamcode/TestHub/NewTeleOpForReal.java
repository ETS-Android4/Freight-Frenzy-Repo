package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass;

@TeleOp
public class NewTeleOpForReal extends LinearOpMode {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    VPivotClass VPivotClass = new VPivotClass();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();


        while (opModeIsActive()) {
            robot.TP_M.setPower(VPivotClass.NEWVPivot(1500, 16, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(), robot.TP_G.getState(), getRuntime(), 16, 0, 0, 0, 0, .2));

        }
    }
}