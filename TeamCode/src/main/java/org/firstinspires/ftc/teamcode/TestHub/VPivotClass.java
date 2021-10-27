package org.firstinspires.ftc.teamcode.TestHub;

public class VPivotClass {

    double vPivotMin = 0.8, vPivotMax = 3, vPivotSet = 1.15;
    double vPivotDifference = 0, vPivotMultipliedP = 0, vPivotP = 3, vPivotD = 0, vPivotMultipliedD, FinalMotorPower;
    double lastError;

    public double VPivotMethod(double Controller, double POTReading){

        vPivotSet = vPivotSet + (.02 * Controller);

        //VPivot Limits
        if(vPivotSet < vPivotMin){
            vPivotSet = vPivotMin;
        }else if(vPivotSet > vPivotMax){
            vPivotSet = vPivotMax;
        }

        vPivotDifference = POTReading - vPivotSet;
        vPivotMultipliedP = vPivotDifference * vPivotP;
        vPivotMultipliedD = (vPivotDifference - lastError)* vPivotD;

        FinalMotorPower = vPivotMultipliedP + vPivotMultipliedD;

        lastError = vPivotDifference;
        return FinalMotorPower;
    }
}
