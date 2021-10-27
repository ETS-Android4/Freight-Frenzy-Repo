package org.firstinspires.ftc.teamcode.TestHub;

public class ExtendClass {
    double extendMin = 2, extendMax = 1300, extendSet = extendMin;
    double extendDifference = 0, extendMultipliedP = 0, extendP = -.02, extendD = 0, extendMultipliedD = 0;

    double ExtendMotorPower = 0, lastError = 0;

    public double ExtendMethod(double Controller, double extendEncoder, boolean MagneticExtend){
        extendSet = extendSet + (15 * (Controller));


        if(MagneticExtend == true){
            extendMin = extendEncoder;
        }

        //Setpoint limits
        if(extendSet < extendMin){
            extendSet = (extendMin + 2);
        }else  if( extendSet > extendMax){
            extendSet = (extendMax - 2);
        }

        extendDifference = extendEncoder - extendSet;
        extendMultipliedP = extendDifference * extendP;
        extendMultipliedD = (extendEncoder - lastError) * extendD;
        ExtendMotorPower = extendMultipliedP + extendMultipliedD;

        lastError = extendDifference;
        return ExtendMotorPower;
    }

}
