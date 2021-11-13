package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class FreightFrenzyHardwareMap {
    // Motors and Servos
    public DcMotor LF_M;
    public DcMotor LB_M;
    public DcMotor RF_M;
    public DcMotor RB_M;
    public DcMotor TR_M;
    public DcMotor TP_M;
    public DcMotor TE_M;
    public CRServo RI_S;
    public CRServo LI_S;
    public AnalogInput TP_P;
    DigitalChannel TR_G;
    DigitalChannel TE_G;


    HardwareMap TestHubHardware;

    public void init(HardwareMap TestHubHardware){

        // Define motors and servos
        LF_M = TestHubHardware.get(DcMotor.class, "LF_M");
        LB_M = TestHubHardware.get(DcMotor.class, "LB_M");
        RF_M = TestHubHardware.get(DcMotor.class, "RF_M");
        RB_M = TestHubHardware.get(DcMotor.class, "RB_M");
        TR_M = TestHubHardware.get(DcMotor.class, "TR_M");
        TP_M = TestHubHardware.get(DcMotor.class, "TP_M");
        TE_M = TestHubHardware.get(DcMotor.class, "TE_M");
        RI_S = TestHubHardware.get(CRServo.class, "RI_S");
        LI_S = TestHubHardware.get(CRServo.class, "LI_S");
        TP_P = TestHubHardware.get(AnalogInput.class, "TP_P");
        TR_G = TestHubHardware.get(DigitalChannel.class, "TR_G");
        TE_G = TestHubHardware.get(DigitalChannel.class, "TE_G");

        // ColorSensor1 = JustmotorhardwareMap.get(NormalizedColorSensor.class, "ColorSensor1");

        //servo = hardwareMap.get(Servo.class, "servo");

        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TR_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TP_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TE_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TR_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TP_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TE_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TR_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TP_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TE_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TR_G.setMode(DigitalChannel.Mode.INPUT);
        TE_G.setMode(DigitalChannel.Mode.INPUT);
    }

}
