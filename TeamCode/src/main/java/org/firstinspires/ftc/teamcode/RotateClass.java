package org.firstinspires.ftc.teamcode;


public class RotateClass{

    double rotateMin = -5000, rotateMax = 5000, rotateSet = 0;
    double rotateDifference = 0, MultipliedP = 0, rotateP = -.01, rotateD = 0; double lastError = 0; double MultipliedD = 0; double rotateMotorPower;
    boolean lastrotatemag = false; double lastEncoder = 0;

    public double RotateMethod(double Controller, double rotateEncoder, boolean rotateMagnet){

        rotateSet = rotateSet + (16 * Controller);

        //ADD IN MAGNET RESET
        /*if(rotateMagnet == true && lastrotatemag == false){
            if(lastEncoder - rotateEncoder < 0){

            }
        }*/
        //Hpivot Limits
        if(rotateSet < rotateMin){
            rotateSet = rotateMin;
        }else if(rotateSet > rotateMax){
            rotateSet = rotateMax;
        }


        rotateDifference = rotateEncoder - rotateSet;
        MultipliedP = rotateDifference * rotateP;
        MultipliedD = (rotateDifference - lastError) * rotateD;
        rotateMotorPower = MultipliedP + MultipliedD;

        lastError = rotateDifference;
        lastEncoder = rotateEncoder;

        return rotateMotorPower;
    }

    public double RotateAutoMethod(double desiredset, double speed, double rotateEncoder, boolean rotateMagnet){

        rotateSet = desiredset;

        //ADD IN MAGNET RESET
        //Hpivot Limits
        if(rotateSet < rotateMin){
            rotateSet = rotateMin;
        }else if(rotateSet > rotateMax){
            rotateSet = rotateMax;
        }


        rotateDifference = rotateEncoder - rotateSet;
        MultipliedP = rotateDifference * rotateP;
        MultipliedD = (rotateDifference - lastError) * rotateD;
        rotateMotorPower = MultipliedP + MultipliedD;

        lastError = rotateDifference;

        if(rotateMotorPower > speed && rotateMotorPower > .05){
            rotateMotorPower = speed;
        }else if(rotateMotorPower < -speed && rotateMotorPower < .05){
            rotateMotorPower = -speed;
        }
        return rotateMotorPower;
    }
}
