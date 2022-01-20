package org.firstinspires.ftc.teamcode.TurretClasses;



public class ExtendClass {
    //setting variables for later use in the code
    double extendMin = -2000, extendMax = 1500, extendSet = 0;
    double extendDifference = 0, extendMultipliedP = 0, extendP = -.0085, extendD = -0.0095, extendI = 0, extendMultipliedD = 0, encoderTickPerInch = 20;
    double homingnextset; boolean HasExtended = false; double homingMin = 0; boolean isHomed = false, lastmagnetic = false;
    public double ExtendMotorPower = 0, lastError = 0, HomingMotorpower = 0, extendModifiedEncoder = 0, deltaEncoder = 0, lastencoder = 0, extendSpeedSet, extendCurrentSpeed = 0;
    public double extendSpeedDifference = 0, extendSpeedCorrection, lastextendSpeedDifference = 0, extendIntegralsum = 0, extendIntegralMax, lastDesiredSet = 0;

    double Direction;


    public double ExtendSpeedMethod(double desiredset, double SpeedSet, double extendEncoder, boolean MagneticExtend, double time) {

        //setting the desired setpoint and capping the setpoint
        extendSet = desiredset;

        if (extendSet < extendMin) {
            extendSet = (extendMin);
        } else if (extendSet > extendMax) {
            extendSet = (extendMax);
        }

        //reseting the encoder minimum at the magnetic sensor so we stop at the sensor and not overrun
        if (MagneticExtend == false && lastmagnetic == true) {
            extendMin = 0;
            extendModifiedEncoder = 0;
            extendMax = extendMin + 1525;
            lastmagnetic = false;
        }else if(MagneticExtend == false){
            lastmagnetic = false;
        }
        else{
            lastmagnetic = true;
        }

        //Finding how much our encoder changed in between loop cycles and setting our modified encoder value
        deltaEncoder = extendEncoder - lastencoder;
        extendModifiedEncoder = extendModifiedEncoder + deltaEncoder;

        //setting what direction the arm needs to extend
        if(extendSet < extendModifiedEncoder){
            Direction = -1;
        }else if(extendSet > extendModifiedEncoder){
            Direction = 1;
        }

        //setting our speed setpoint positive or negative depending on what direction it needs to go
        extendSpeedSet = Math.copySign(SpeedSet, Direction);

        if(Math.abs(desiredset - extendModifiedEncoder) < 100){
            extendSpeedSet = extendSpeedSet * (Math.abs(desiredset - extendModifiedEncoder));
        }

        //calculating the current speed of the arm
        extendCurrentSpeed = (deltaEncoder/encoderTickPerInch)/time;

        //Calculating how far away the robot are from our speed setpoint
        extendSpeedDifference = extendSpeedSet - extendCurrentSpeed;

        //If the setpoint changed rest the integral windup
        if(desiredset != lastDesiredSet){
            extendIntegralsum = 0;
        }

        //calculating and limiting our intigral
        extendIntegralsum = extendIntegralsum + (extendSpeedDifference * time);

        if(extendIntegralsum > extendIntegralMax){
            extendIntegralsum = extendIntegralMax;
        }else if(extendIntegralsum < -extendIntegralMax){
            extendIntegralsum = -extendIntegralMax;
        }


        ExtendMotorPower = (extendSpeedDifference * extendP) + (extendIntegralsum * extendI) + ((extendSpeedDifference - lastextendSpeedDifference) * extendP);



        lastencoder = extendEncoder;
        lastDesiredSet = desiredset;
        lastextendSpeedDifference = extendSpeedDifference;
        //returning the motor power for use
        return ExtendMotorPower;

    }


    public double ExtendAutoMethod(double desiredset, double speed, double extendEncoder, boolean MagneticExtend) {

        extendSet = desiredset;

        //reseting the encoder minimum at the magnetic sensor so we stop at the sensor and not overrun
        if (MagneticExtend == false && lastmagnetic == true) {
            extendMin = 0;
            extendModifiedEncoder = 0;
            extendMax = extendMin + 1525;
            lastmagnetic = false;
        }else if(MagneticExtend == false){
            lastmagnetic = false;
        }
        else{
            lastmagnetic = true;
        }

        deltaEncoder = extendEncoder - lastencoder;

        extendModifiedEncoder = extendModifiedEncoder + deltaEncoder;

        //Setpoint limits
        if (extendSet < extendMin) {
            extendSet = (extendMin);
        } else if (extendSet > extendMax) {
            extendSet = (extendMax);
        }

        //finding difference so we can use it for the proportional and derivative multipliers
        extendDifference = extendModifiedEncoder - extendSet;
        extendMultipliedP = extendDifference * extendP;//Proportional multiplying
        extendMultipliedD = (extendDifference - lastError) * extendD;//derivative multiplying
        ExtendMotorPower = extendMultipliedP + extendMultipliedD;//adding them together to give 1 motor power

        lastError = extendDifference;// setting last error for use in derivative

        //setting limits for speed to go a desired setpoint
        if(ExtendMotorPower > speed){
            ExtendMotorPower = speed;
        }else if (ExtendMotorPower < -speed ){
            ExtendMotorPower = -speed;
        }

        lastencoder = extendEncoder;
        //returning the motor power for use
        return ExtendMotorPower;

    }


    //A homing program for the extend
    //we need a homing to know where we are at the start of the match
    public double ExtendHoming(boolean ManeticExtend, double Extendencoder){

        //looking to see if the slides are already fully retracted
        if(ManeticExtend == false){
            //dermeines if we started retracted or if we ran the homing sequence
            if(HasExtended == false){
                //setting the setpoint for the extension of homing
                homingnextset = extendModifiedEncoder + 40;
                HomingMotorpower =  ExtendAutoMethod(homingnextset,1, Extendencoder, ManeticExtend);
            }else{
                //sets the minimum
                homingMin = Extendencoder;
                extendModifiedEncoder = 0;
                isHomed = true;
            }

        }else{
            //setting the setpoint for the retraction of homing
            homingnextset = extendModifiedEncoder - 20;
            HomingMotorpower = ExtendAutoMethod(homingnextset,1, Extendencoder, ManeticExtend);
            HasExtended = true;
        }
        return HomingMotorpower;
    }
    //returns the bvarible ishomed so we can know if we have homed in the main loop
    public boolean isHomedExtendReturn(){return isHomed;}
    public double  ExtendMotorPowerReturn(){return ExtendMotorPower;}
    public double HomingnextSetReturn(){return homingnextset;}

}


