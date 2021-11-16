package org.firstinspires.ftc.teamcode.TestHub;

public class VPivotClass {

    //setting variables for use in later code
    double vPivotMin = 0.8, vPivotMax = 3, vPivotSet = 1.15;
    double vPivotDifference = 0, vPivotMultipliedP = 0, vPivotP = 3.85, vPivotD = 3.5, vPivotMultipliedD; // 6.5, 6
    public double FinalMotorPower;
    double lastError;

    //main TeleOp code for vertical Pivot
    public double VPivotMethod(double Controller, double POTReading){

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
        FinalMotorPower = FinalMotorPower + vPivotMultipliedP + vPivotMultipliedD;

        lastError = vPivotDifference;//setting last error for use next loop cycle

        //setting speed to go at a set speed and not go to fast and break something
        if(FinalMotorPower > speed){
            FinalMotorPower = speed;
        }else if(FinalMotorPower < -speed){
            FinalMotorPower = -speed;
        }

        return FinalMotorPower; //outputting a motor power when called
    }
    public double PivotSetReturn(){return vPivotSet; }
}
