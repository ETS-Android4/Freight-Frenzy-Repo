package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretTesting {
    // Motors and Servos

    public DcMotor TR_M;
    public DcMotor TP_M;
    public DcMotor TE_M;
   // public CRServo RI_S;
    //public CRServo LI_S;
    public AnalogInput TP_P;
    public DigitalChannel TR_G;
    public DigitalChannel TE_G;


    HardwareMap TurretTestingHardware;

    public void init(HardwareMap TurretTestingHardware){

        // Define motors and servos

        TR_M = TurretTestingHardware.get(DcMotor.class, "TR_M");
        TP_M = TurretTestingHardware.get(DcMotor.class, "TP_M");
        TE_M = TurretTestingHardware.get(DcMotor.class, "TE_M");
       // RI_S = TurretTestingHardware.get(CRServo.class, "RI_S");
       // LI_S = TurretTestingHardware.get(CRServo.class, "LI_S");
        TP_P = TurretTestingHardware.get(AnalogInput.class, "TP_P");
        TR_G = TurretTestingHardware.get(DigitalChannel.class, "TR_G");
        TE_G = TurretTestingHardware.get(DigitalChannel.class, "TE_G");

        // ColorSensor1 = JustmotorhardwareMap.get(NormalizedColorSensor.class, "ColorSensor1");

        //servo = hardwareMap.get(Servo.class, "servo");

        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        TR_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TP_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TE_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TR_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TP_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TE_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TR_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TP_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TE_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TR_G.setMode(DigitalChannel.Mode.INPUT);
        TE_G.setMode(DigitalChannel.Mode.INPUT);
    }

}
