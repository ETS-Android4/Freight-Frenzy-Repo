package org.firstinspires.ftc.teamcode.TurretClasses;


import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;

public class ExtendClass {
    //setting variables for later use in the code
    double extendMin = -2000, extendMax = 1700, extendSet = 0;
    public double extendDifference = 0, extendMultipliedP = 0, extendP = .035, extendD = 0.01, extendI = 0, extendMultipliedD = 0, encoderTickPerInch = 70;
    double homingnextset; boolean HasExtended = false; double homingMin = 0; boolean isHomed = false, lastmagnetic = false;
    public double ExtendMotorPower = 0, lastError = 0, HomingMotorpower = 0, extendModifiedEncoder = 0, deltaEncoder = 0, lastencoder = 0, extendSpeedSet, extendCurrentSpeed = 0;
    public double extendSpeedDifference = 0, extendSpeedCorrection, lastextendSpeedDifference = 0, lastDesiredSet = 0;

    double Direction, timeInSec = 0, lastTimeInSec = 0;

FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();

    public double ExtendSpeedMethod(double desiredset, double SpeedSet, boolean Mag, double Encoder) {

        timeInSec = robot.TimerCustom();

        //setting the desired setpoint and capping the setpoint
        extendSet = desiredset;

        if (extendSet < extendMin) {
            extendSet = (extendMin);
        } else if (extendSet > extendMax) {
            extendSet = (extendMax);
        }

        //reseting the encoder minimum at the magnetic sensor so we stop at the sensor and not overrun
      /*  if (Mag == false && lastmagnetic == true) {
            extendMin = 0;
            extendModifiedEncoder = 0;
            extendMax = extendMin + 1525;
            lastmagnetic = false;
        }else if(Mag == false){
            lastmagnetic = false;
        }
        else{
            lastmagnetic = true;
        }*/

        //Finding how much our encoder changed in between loop cycles and setting our modified encoder value
        deltaEncoder = Encoder - lastencoder;
        extendModifiedEncoder = extendModifiedEncoder + deltaEncoder;

        //setting what direction the arm needs to extend
        if(extendSet < extendModifiedEncoder){
            Direction = -1;
        }else if(extendSet > extendModifiedEncoder){
            Direction = 1;
        }

        //setting our speed setpoint positive or negative depending on what direction it needs to go
        extendSpeedSet = Math.copySign(SpeedSet, Direction);//Max speed is 35 in/second

        if(Math.abs(desiredset - extendModifiedEncoder) < Math.abs(SpeedSet * 13)){
            extendSpeedSet = extendSpeedSet * (Math.abs(desiredset - extendModifiedEncoder)/Math.abs(SpeedSet * 13));
        }

        //calculating the current speed of the arm
        extendCurrentSpeed = (deltaEncoder/encoderTickPerInch)/(timeInSec - lastTimeInSec);

        //Calculating how far away the robot are from our speed setpoint
        extendSpeedDifference = extendSpeedSet - extendCurrentSpeed;


        ExtendMotorPower = ExtendMotorPower + (extendSpeedDifference * extendP) + ((extendSpeedDifference - lastextendSpeedDifference) * extendD);

        if(ExtendMotorPower > 1){
            ExtendMotorPower = 1;
        }else if (ExtendMotorPower < -1){
            ExtendMotorPower = -1;
        }

        lastencoder = Encoder;
        lastTimeInSec = timeInSec;
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
        extendMultipliedP = extendDifference * -.0085;//Proportional multiplying
        extendMultipliedD = (extendDifference - lastError) * -0.0095;//derivative multiplying
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


