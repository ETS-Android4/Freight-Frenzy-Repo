package org.firstinspires.ftc.teamcode.TestHub;



public class ExtendClass {
    //setting variables for later use in the code
    double extendMin = -2000, extendMax = 1450, extendSet = 0;
    double extendDifference = 0, extendMultipliedP = 0, extendP = -.02, extendD = 0, extendMultipliedD = 0;
    double homingnextset; boolean HasExtended = false; double homingMin = 0; boolean isHomed = false, lastmagnetic = false;
    public double ExtendMotorPower = 0, lastError = 0, HomingMotorpower = 0, extendModifiedEncoder = 0, deltaEncoder = 0, lastencoder = 0;

    public double ExtendMethod(double Controller, double extendEncoder, boolean MagneticExtend){
        extendSet = extendSet + (30 * (Controller));//setting the setpoint using the controller input

        //reseting the encoder minimum at the magnetic sensor so we stop at the sensor and not overrun
        if (MagneticExtend == false && lastmagnetic == true) {
            extendMin = 0;
            extendModifiedEncoder = 0;
            extendMax = extendMin + 1400;
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
        if(extendSet < extendMin){
            extendSet = (extendMin + 2);
        }else  if( extendSet > extendMax){
            extendSet = (extendMax - 2);
        }

        //finding difference so we can use it for the proportional and derivative multipliers
        extendDifference = extendModifiedEncoder - extendSet;
        extendMultipliedP = extendDifference * extendP;//Proportional multiplying
        extendMultipliedD = (extendDifference - lastError) * extendD;//derivative multiplying
        ExtendMotorPower = extendMultipliedP + extendMultipliedD;//adding them together to give 1 motor power

        lastError = extendDifference;// setting last error for use in derivative

        lastencoder = extendEncoder;

        //returning the motor power for use
        return ExtendMotorPower;
    }

    public double ExtendAutoMethod(double desiredset, double speed, double extendEncoder, boolean MagneticExtend) {

        extendSet = desiredset;

        //reseting the encoder minimum at the magnetic sensor so we stop at the sensor and not overrun
        if (MagneticExtend == false && lastmagnetic == true) {
            extendMin = 0;
            extendModifiedEncoder = 0;
            extendMax = extendMin + 1450;
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
                    homingnextset = extendModifiedEncoder + 20;
                    HomingMotorpower =  ExtendAutoMethod(homingnextset,.5, Extendencoder, ManeticExtend);
                }else{
                    //sets the minimum
                    homingMin = Extendencoder;
                    extendModifiedEncoder = 0;
                    isHomed = true;
                }

        }else{
            //setting the setpoint for the retraction of homing
            homingnextset = extendModifiedEncoder - 20;
            HomingMotorpower = ExtendAutoMethod(homingnextset,.5, Extendencoder, ManeticExtend);
            HasExtended = true;
        }
        return HomingMotorpower;
    }
    //returns the bvarible ishomed so we can know if we have homed in the main loop
    public boolean isHomedExtendReturn(){return isHomed;}
    public double  ExtendMotorPowerReturn(){return ExtendMotorPower;}
    public double HomingnextSetReturn(){return homingnextset;}

}


