package org.firstinspires.ftc.teamcode.TestHub;

public class VPivotClass {

    double vPivotMin = 0.8, vPivotMax = 3, vPivotSet = 1.15;
    double vPivotDifference = 0, vPivotMultipliedP = 0, vPivotP = 3, vPivotD = 0, vPivotMultipliedD, FinalMotorPower;
    double lastError;

    public double VPivotMethod(double Controller, double POTReading){

        vPivotSet = vPivotSet + (.02 * Controller);//setting the setpoint using the controller input

        //VPivot Limits
        if(vPivotSet < vPivotMin){
            vPivotSet = vPivotMin;
        }else if(vPivotSet > vPivotMax){
            vPivotSet = vPivotMax;
        }

        vPivotDifference = POTReading - vPivotSet;//finding the difference of setpoint to where we are for using below
        vPivotMultipliedP = vPivotDifference * vPivotP;//proportional multiplying to correct to setpoint
        vPivotMultipliedD = (vPivotDifference - lastError)* vPivotD;//derivative multiplying to even out the correction

        FinalMotorPower = vPivotMultipliedP + vPivotMultipliedD;//adding the two together to get 1 motor output

        lastError = vPivotDifference;//setting last error for use next loop cycle

        //outputting a motor power when called
        return FinalMotorPower;
    }

    public double VPivotAutoMethod(double desiredsetpoint, double speed, double POTReading){

        vPivotSet = desiredsetpoint;//setting the setpoint

        //VPivot Limits
        if(vPivotSet < vPivotMin){
            vPivotSet = vPivotMin;
        }else if(vPivotSet > vPivotMax){
            vPivotSet = vPivotMax;
        }

        vPivotDifference = POTReading - vPivotSet;//finding the difference of setpoint to where we are for using below
        vPivotMultipliedP = vPivotDifference * vPivotP;//proportional multiplying to correct to setpoint
        vPivotMultipliedD = (vPivotDifference - lastError)* vPivotD;//derivative multiplying to even out the correction

        FinalMotorPower = vPivotMultipliedP + vPivotMultipliedD;//adding the two together to get 1 motor output

        lastError = vPivotDifference;//setting last error for use next loop cycle

        //setting speed to go at a set speed and not go to fast and break something
        if(FinalMotorPower > speed && FinalMotorPower > .05){
            FinalMotorPower = speed;
        }else if(FinalMotorPower < -speed && FinalMotorPower < -.05){
            FinalMotorPower = -speed;
        }

        //outputting a motor power when called
        return FinalMotorPower;
    }
}
