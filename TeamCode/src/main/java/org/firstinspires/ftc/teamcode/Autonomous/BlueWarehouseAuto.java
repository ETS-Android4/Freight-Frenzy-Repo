package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.DirectionCalcClass;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.Odometry;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.SpeedClass;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.TurnControl;
import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.TestHub.OpenCVTest;
import org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass;
import org.firstinspires.ftc.teamcode.TurretClasses.RotateClass;
import org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous

public class BlueWarehouseAuto extends LinearOpMode {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    SpeedClass SpeedClass = new SpeedClass();
    DirectionCalcClass DirectionClass = new DirectionCalcClass();
    TurnControl TurnControl = new TurnControl();
    Odometry OdoClass = new Odometry();
    ExtendClass ExtendClass = new ExtendClass();
    VPivotClass VPivotClass = new VPivotClass();
    RotateClass RotateClass = new RotateClass();
    //Uses Vuforia Developer Code
    //Declares Varibles
    double breakout;
    double Detected;
    double startPointX;
    double startPointY;
    double lastEndPointY;
    double justTurn;
    double timepassed;
    double lastEndPointX;
    double xSetpoint;
    double ySetpoint;
    double turnIncriments;
    double onlyDriveMotors;
    double thetaSetpoint;
    double loopcount;
    double accelerationDistance;
    double slowMoveSpeed;
    double decelerationDistance;
    double slowMovedDistance;
    double thetaTargetSpeed;
    double thetaDeccelerationDegree;
    double targetSpeed;
    double stopProgram;
    double rotateSetpoint;
    double rotateSpeed;
    double extendSetpoint;
    double extendSpeed;
    double VPivotSetpoint;
    double VPivotSpeed;
    double TSEPos;
    double timepassed2;
    public static double UPARMPM = .009;
    public static double UPARMDM = .008;
    public static double DNPM = .008;
    public static double DNDM = .005;
    public  static double SPEEDSET = 16;
    public static double MINSPEED = .2;
    public static double SETPOINT = 1500;

    double action;
    double initPOsitionOrder = 1;
    OpenCvCamera webcam;
    static OpenCVTest.TestPipeline pipeline;
    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //allows to call pipline
        pipeline = new OpenCVTest.TestPipeline();
        //sets the webcam to the pipeline
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            //starts the webcam and defines the pixels
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        telemetry.update();
        //Depending on the ring stack we change our intake to diffrent heights to be able to reach the top of the stack
        //Enters our 1 loop system, will exit once all actions are done
        while (!opModeIsActive()){
            if(RotateClass.isHomedRotateReturn() == false){
                if(VPivotClass.has1stloop == true){
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(2000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));
                    if(VPivotClass.encoderWithOffset > 1900 && VPivotClass.encoderWithOffset < 2100){
                        robot.TE_M.setPower(ExtendClass.ExtendHoming(robot.TE_G.getState(), robot.TE_M.getCurrentPosition()));
                        if(ExtendClass.isHomedExtendReturn() == true){
                            robot.TR_M.setPower(RotateClass.RotateHoming(robot.TR_G.getState(), robot.TR_M.getCurrentPosition()));
                        }
                    }
                }else{
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(3000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));
                }

            }else{
                telemetry.addData("homed",0);
                robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(10,.8,robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                if(initPOsitionOrder == 1){
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(2000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(800,.8,robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                    if(RotateClass.modifiedRotateCurrent() > 750 && RotateClass.modifiedRotateCurrent() < 850){
                        initPOsitionOrder = 2;
                    }
                }else if(initPOsitionOrder == 2){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(800,.8,robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(500, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));
                    if(VPivotClass.encoderWithOffset < 550 && VPivotClass.encoderWithOffset > 450){
                        initPOsitionOrder = 3;
                    }
                }else if(initPOsitionOrder == 3){
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(650,.4,robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                    robot.TP_M.setPower(VPivotClass.NEWVPivot(500, 5, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(),robot.TP_G.getState(), getRuntime(), 16, UPARMPM,UPARMDM,DNPM,DNDM, MINSPEED));

                }



            }
            if(pipeline.region1Avg() <= 125){
             TSEPos = 2;
             telemetry.addData("TSE", 2);
            }
            else if(pipeline.region2Avg() <= 125){
                TSEPos = 3;
                telemetry.addData("TSE", 3);
            }
            else{
                TSEPos = 1;
                telemetry.addData("TSE", 1);
            }
            telemetry.update();
        }



        waitForStart();
        robot.LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shuts down Tensor Flow
        //Sets our intial varible setpoints
        action = 1;
        startPointX = 0;
        startPointY = 0;
        stopProgram = 0;
        xSetpoint = 0;
        ySetpoint = 0;
        thetaSetpoint = 0;
        targetSpeed = 3;
        accelerationDistance = 0;
        decelerationDistance = 2;
        slowMoveSpeed = 3.85;
        slowMovedDistance = 1;
        thetaDeccelerationDegree = 1;
        thetaTargetSpeed = .4;
        rotateSpeed = .325;
        extendSpeed = .4;
        VPivotSpeed = .25;
        rotateSetpoint = 1550;
        extendSetpoint = 0;
        VPivotSetpoint = 1.2;
        while (opModeIsActive() && stopProgram == 0) {

            if (action == 1) {
                if ((robot.TP_P.getVoltage() >= 1.15) && (robot.TP_P.getVoltage() <= 1.25)) {
                    StopMotors();
                    action = 2;
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                } else {
                    breakout = 1;
                }
            } else if (action == 2) {
                rotateSetpoint = 1550;
                if (RotateClass.modifiedRotateCurrent() <= 1600 && RotateClass.modifiedRotateCurrent() >= 1500) {
                    action = 3;
                    breakout = 0;
                } else {
                    breakout = 1;
                    timepassed = 1;
                }
            } else if (action == 3) {
                extendSetpoint = 1450;
                if ((ExtendClass.extendModifiedEncoder <= 1500 && ExtendClass.extendModifiedEncoder >= 1400)) {
                    action = 4;
                    breakout = 0;
                } else {
                    breakout = 1;
                    timepassed = 1;
                }

            } else if (action == 4) {
                robot.LI_S.setPower(-.7);
                robot.RI_S.setPower(.7);
                if (timepassed == 1) {
                    timepassed2 = getRuntime();
                    timepassed = 0;
                }
                if (timepassed2 + 3.5 <= getRuntime()) {
                    action = 5;
                    breakout = 0;
                    timepassed = 1;
                } else {
                    breakout = 1;

                }
            } else if (action == 5) {
                robot.LI_S.setPower(0);
                robot.RI_S.setPower(0);
                extendSetpoint = 0;
                if ((ExtendClass.extendModifiedEncoder <= 100)) {
                    action = 6;
                    breakout = 0;
                } else {
                    breakout = 1;
                    timepassed = 1;
                }


                ///Moves to first power shot shooting position

                //Exits once the robot is a certain distance and angle away

            } else if (action == 6) {
                    thetaSetpoint = 0;
                    accelerationDistance = .04;
                    decelerationDistance = 8;
                    slowMoveSpeed = 3.85;
                    slowMovedDistance = 1;
                    thetaDeccelerationDegree = 2;
                    thetaTargetSpeed = 1;
                    VPivotSetpoint = .9;
                    VPivotSpeed = .3;
                    xSetpoint = 50;
                    ySetpoint = .8;
                    thetaSetpoint = 0;
                    targetSpeed = 20;
                    rotateSpeed = .4;
                    rotateSetpoint = 0;
                    //Exits once the robot is a certain distance and angle away
                if (DirectionClass.distanceFromReturn() <= .5 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < .5 && OdoClass.thetaInDegreesReturn() > -.5)) {
                        StopMotors();
                        action = 7;
                        startPointX = OdoClass.odoXReturn();
                        startPointY = OdoClass.odoYReturn();
                        breakout = 0;
                    } else {
                        breakout = 1;
                    }
                }
           else if (action == 7) {
                xSetpoint = 50;
                ySetpoint = 25;
                thetaSetpoint = 0;
                targetSpeed = 15;
                //Exits once the robot is a certain distance and angle away
                if (DirectionClass.distanceFromReturn() <= .5 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < .5 && OdoClass.thetaInDegreesReturn() > -.5)) {
                    StopMotors();
                    action = 8;
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                } else {
                    breakout = 1;
                }
            }




            /*else if (action == 2) {
                xSetpoint = 30;
                ySetpoint = 0;
                thetaSetpoint = 0;
                targetSpeed = 40;
                //Exits once the robot is a certain distance and angle away
                if (DirectionClass.distanceFromReturn() <= .5 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < .5 && OdoClass.thetaInDegreesReturn() > -.5)) {
                    StopMotors();
                    action = 3;
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                } else {
                    breakout = 1;
                }
            }
            */
            //If nothing else to do, stop the program
            else {
                stopProgram = 1;
                StopMotors();
            }
            //Runs all of our equations each loop cycle
            Movement(xSetpoint, ySetpoint, thetaSetpoint, targetSpeed, thetaTargetSpeed, thetaDeccelerationDegree, slowMoveSpeed, accelerationDistance, decelerationDistance, slowMovedDistance);
            RotateClass.RotateAutoMethod(rotateSetpoint, rotateSpeed, robot.TR_M.getCurrentPosition(), robot.TR_G.getState());
            ExtendClass.ExtendAutoMethod(extendSetpoint, extendSpeed, robot.TE_M.getCurrentPosition(), robot.TE_G.getState());
            //VPivotClass.VPivotAutoMethod(VPivotSetpoint, VPivotSpeed, robot.TP_P.getVoltage());
            PowerSetting();
            Telemetry();
        }
    }


    public void Telemetry() {
        //Displays telemetry
        telemetry.addData("Odo X", OdoClass.odoXReturn());
        telemetry.addData("Odo Y", OdoClass.odoYReturn());
        telemetry.addData("Theta Angle", OdoClass.thetaInDegreesReturn());
        telemetry.addData("X", DirectionClass.XReturn());
        telemetry.addData("Y", DirectionClass.YReturn());
        telemetry.addData("Theta", TurnControl.theta);
        telemetry.addData("ThetaSpeedSetpoint", SpeedClass.thetaSpeedSetpoint());
        telemetry.addData("SlowMoveSpeed", slowMoveSpeed);
        telemetry.addData("slowMovedDistance", slowMovedDistance);
        telemetry.addData("Distance", DirectionClass.distanceReturn());
        telemetry.addData("Distance From", DirectionClass.distanceFromReturn());
        telemetry.addData("Speed Setpoint", SpeedClass.speedSetpoint());
        telemetry.addData("Speed", SpeedClass.SpeedReturn());
        telemetry.addData("time", getRuntime());
        telemetry.addData("Distance Delta", SpeedClass.DistanceDelta());
        telemetry.addData("XSetpoint", DirectionClass.XSetpointReturn());
        telemetry.addData("YSetpoint", DirectionClass.YSetpointReturn());
        telemetry.addData("LF_Power", robot.LF_M.getPower());
        telemetry.addData("RF_Power", robot.RF_M.getPower());
        telemetry.addData("LF_Direction", DirectionClass.LF_M_DirectionReturn());
        telemetry.addData("Motor Power Ratio", DirectionClass.motorPowerRatioReturn());
        telemetry.addData("Action", action);
        telemetry.addData("PT", robot.TP_P.getVoltage());
        telemetry.addData("ExtendE", ExtendClass.extendModifiedEncoder);
        telemetry.addData("RotateE", RotateClass.modifiedRotateCurrent());
        telemetry.update();
    }

    public void Movement(double endpointx, double endpointy, double thetasetpoint, double targetspeed, double thetaTargetSpeed, double thetaDeccelerationDegree, double slowMoveSpeed, double accelerationdistance, double deccelerationdistance, double slowMovedDistance) {
        OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
        TurnControl.turnControl(thetaSetpoint, OdoClass.thetaInDegreesReturn());
        DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
        SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, slowMovedDistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn(), slowMoveSpeed, thetaDeccelerationDegree, thetasetpoint, thetaTargetSpeed, OdoClass.thetaInDegreesReturn());
        SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), OdoClass.thetaInDegreesReturn(), getRuntime(), SpeedClass.speedSetpoint(), SpeedClass.thetaSpeedSetpoint());
    }

    public void PowerSetting() {

        robot.TR_M.setPower(RotateClass.rotateMotorPower);
        robot.TE_M.setPower(ExtendClass.ExtendMotorPower);
       robot.TP_M.setPower(VPivotClass.FinalMotorPower);
        robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.SpeedReturn()));
        robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.SpeedReturn() ));
       robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.SpeedReturn()));
       robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.SpeedReturn() ));


    }

    public void StopMotors() {
        robot.LF_M.setPower(0);
        robot.LB_M.setPower(0);
        robot.RF_M.setPower(0);
        robot.RB_M.setPower(0);
    }
   public static class testPipeline extends OpenCvPipeline {

        //sets colors for the boxes that we will look in
       static final Scalar CRIMSON = new Scalar(220, 20, 60);
       static final Scalar AQUA = new Scalar(79, 195, 247);
      // static final Scalar PARAKEET = new Scalar(3, 192, 74);

        //sets the boxes where we will look
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(270, 260);
        static int REGION1_WIDTH = 60;
        static int REGION1_HEIGHT = 85;
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(430, 260);
        static final int REGION2_WIDTH = 60;
        static final int REGION2_HEIGHT = 85;
        //static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(430, 260);
        //static final int REGION3_WIDTH = 60;
        //static final int REGION3_HEIGHT = 85;
        //static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(155, 260);
       //static int REGION1_WIDTH = 60;
       //static int REGION1_HEIGHT = 85;

        //uses the boxes setpoints and makes the ctual box
        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

        Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

       // Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
        //Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);


        Mat region1_G, region2_G, region3_G, region4_G;
        Mat RGBA = new Mat();
        Mat HLS = new Mat();
        Mat A = new Mat();
        Mat H = new Mat();
        int avg1, avg2, avg3, avg4;


        //actual image processing
        void inputToG(Mat input) {
            Imgproc.cvtColor(input, RGBA, Imgproc.COLOR_RGB2RGBA);
            Core.extractChannel(RGBA, A, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToG(firstFrame);
            //sets region to look in for color
            region1_G = A.submat(new Rect(region1_pointA, region1_pointB));
            region2_G = A.submat(new Rect(region2_pointA, region2_pointB));
            //region3_G = A.submat(new Rect(region3_pointA, region3_pointB));
            //region4_G = L.submat(new Rect(region4_pointA, region4_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {

            inputToG(input);

            avg1 = (int) Core.mean(region1_G).val[0];
            avg2 = (int) Core.mean(region2_G).val[0];
            //avg3 = (int) Core.mean(region3_G).val[0];
            //avg4 = (int) Core.mean(region4_G).val[0];

            Imgproc.rectangle(input, region1_pointA, region1_pointB, CRIMSON, 2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, AQUA, 2);
           // Imgproc.rectangle(input, region3_pointA, region3_pointB, PARAKEET, 2);
            //Imgproc.rectangle(input, region4_pointA, region4_pointB, GOLD, 2);

            return input;
        }

        public int region1Avg() {
            return avg1;
        }

        public int region2Avg() {
            return avg2;
        }

        //public int region3Avg() { return avg3; }



    }
}