package org.firstinspires.ftc.teamcode.TurretClasses;


public class VPivotClass {

    //setting variables for use in later code
    double vPivotMin = 0.8, vPivotMax = 3, vPivotSet = 1.15;
    double vPivotDifference = 0, vPivotMultipliedP = 0, vPivotP = 3, vPivotD = 0, vPivotMultipliedD; // 6.5, 6
    public double FinalMotorPower, PDMultipliedMotorPower, lastset;
    double lastError;
    double lastTime, lastPOT, speed, speedDifference, speedCorrection, speedPM = 1, speedDM = 0, lastSpeedDifference = 0, vPivotPOT1degree = .0105, vPivotEncoder1Degree = 23;//TODO correct this variable
    double degreesTraveled, speedSetPoint;
    public double vPivotCorrection = 0, lastvPivotError, deltaPivot, encoderWithOffset, deltaPivotEncoder, lastEncoder = 0;
    boolean has1stloop = false;

    //Mew method for better control of the turret
    public double NEWVPivot(double SetPoint, double SpeedSetPt ,double POTReading, double EncoderReading, double time, double extendlength){

        //setting set point limits on the arm to prevent the arm from over rotating
        if(SetPoint < -3000){//minimum travel
            vPivotSet = -3000;
        }else if (SetPoint > 0){//maximum travel
            vPivotSet = 0;
        }

        //setting our encoder offset readings, using a potentiometer for the starting number
        if(has1stloop == false){
            encoderWithOffset = (POTReading - 2.092) / .00045;
            has1stloop = true;
        }
            deltaPivotEncoder = EncoderReading - lastEncoder;
            encoderWithOffset = encoderWithOffset + deltaPivotEncoder;


        //Calculating Speed at tip of arm

        degreesTraveled = deltaPivotEncoder / vPivotEncoder1Degree;
        speed = (((2 * extendlength) * Math.PI)*(degreesTraveled / 360)) / (time - lastTime);

        //Motion Profile
        if(Math.abs(SetPoint) - Math.abs(EncoderReading) < 100){
            speedSetPoint = SpeedSetPt * ((Math.abs(SetPoint) - Math.abs(EncoderReading))/100);
        }else{
            speedSetPoint = SpeedSetPt;
        }

        //Speed Proportional and derivative correction equations to make the arm move at a desired speed not motor power
        speedDifference = speedSetPoint - speed;
        speedCorrection = (speedDifference * speedPM) + ((speedDifference - lastSpeedDifference) * speedDM);

        //Vertical Pivot Proportional and Derivative correction equations to correct the arm Position to the desired set point
        vPivotDifference = vPivotSet - encoderWithOffset;
        vPivotCorrection = (vPivotDifference * vPivotP) + ((vPivotDifference - lastvPivotError) * vPivotD);

        //combining the vPivot correction equation and the speed correction equation for and accurate motor power
        FinalMotorPower = vPivotCorrection + speedCorrection;

        //setting variables for use in the next loop cycle
        lastTime = time;
        lastPOT = POTReading;
        lastSpeedDifference = speedDifference;
        lastvPivotError = vPivotDifference;
        lastEncoder = EncoderReading;

        return FinalMotorPower;
    }

    //main TeleOp code for vertical Pivot
    public double VPivotMethod(double Controller, double POTReading){
        //Setting and limiting setpoints

            //setting the setpoint using the controller input
            vPivotSet = Controller ;//+ (.03 * Controller);

            //VPivot Limits
            if(vPivotSet < vPivotMin){
                vPivotSet = vPivotMin;
            }else if(vPivotSet > vPivotMax){
                vPivotSet = vPivotMax;
            }

        //Correction Loop

            //finding the difference of setpoint to where we are for using below
            vPivotDifference = POTReading - vPivotSet;
            //proportional multiplying to correct to setpoint
            vPivotMultipliedP = vPivotDifference * vPivotP;
            //derivative multiplying to even out the correction
            vPivotMultipliedD = (vPivotDifference - lastError)* vPivotD;
            FinalMotorPower = .3 +( vPivotMultipliedP + vPivotMultipliedD);
            if(FinalMotorPower > 1){
                FinalMotorPower = 1;
            }else if(FinalMotorPower < -1){
                FinalMotorPower = -1;
        }


        //setting last error for use next loop cycle
        lastError = vPivotDifference;

        return FinalMotorPower;
    }

    public double OldVPivotMethod(double Controller, double POTReading){

        //setting the setpoint using the controller input
        vPivotSet = vPivotSet + (.03 * Controller);

        //VPivot Limits
        if(vPivotSet < vPivotMin){
            vPivotSet = vPivotMin;
        }else if(vPivotSet > vPivotMax){
            vPivotSet = vPivotMax;
        }

        //finding the difference of setpoint to where we are for using below
        vPivotDifference = POTReading - vPivotSet;
        //proportional multiplying to correct to setpoint
        vPivotMultipliedP = vPivotDifference * vPivotP;
        //derivative multiplying to even out the correction
        vPivotMultipliedD = (vPivotDifference - lastError)* vPivotD;
        //adding the two together to get 1 motor output
        FinalMotorPower = vPivotMultipliedP + vPivotMultipliedD;

        //setting last error for use next loop cycle
        lastError = vPivotDifference;

        //outputting a motor power when called
        return FinalMotorPower;
    }

    //Autonomous Code for vertical pivoting
    public double VPivotAutoMethod(double desiredsetpoint, double speed, double POTReading){

        vPivotSet = desiredsetpoint;//setting the setpoint

        //VPivot Limits
        if(vPivotSet < vPivotMin){
            vPivotSet = vPivotMin;
        }else if(vPivotSet > vPivotMax){
            vPivotSet = vPivotMax;
        }

        //finding the difference of setpoint to where we are for using below
        vPivotDifference = POTReading - vPivotSet;
        vPivotMultipliedP = vPivotDifference * vPivotP; //proportional multiplying to correct to setpoint
        vPivotMultipliedD = (vPivotDifference - lastError)* vPivotD;//derivative multiplying to even out the correction

        //adding the two together to get 1 motor output
        PDMultipliedMotorPower = vPivotMultipliedP + vPivotMultipliedD;

        if(vPivotSet < POTReading){
            FinalMotorPower = Math.abs(PDMultipliedMotorPower);
        }else if(vPivotSet > POTReading){
            FinalMotorPower = Math.abs(PDMultipliedMotorPower)*-.3;
        }

        //FinalMotorPower = FinalMotorPower + vPivotMultipliedP + vPivotMultipliedD;

        lastError = vPivotDifference;//setting last error for use next loop cycle

        //setting speed to go at a set speed and not go to fast and break something
        /*
        if(FinalMotorPower > speed){
            FinalMotorPower = speed;
        }else if(FinalMotorPower < -speed){
            FinalMotorPower = -speed;
        }


         */
        lastset = vPivotSet;

        return FinalMotorPower; //outputting a motor power when called
    }
    public double PivotSetReturn(){return vPivotSet; }
    public double SpeedReturn(){return  speed;}
    public double DegreesTravelReturn(){ return degreesTraveled;}
    public double LastPOTreadingReturn(){return lastPOT;}
}
