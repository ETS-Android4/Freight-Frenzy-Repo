package org.firstinspires.ftc.teamcode;


public class RotateClass{

    double rotateMin = -5000, rotateMax = 5000, rotateSet = 0;
    double rotateDifference = 0, MultipliedP = 0, rotateP = -.01, rotateD = 0; double lastError = 0; double MultipliedD = 0; double rotateMotorPower;
    boolean lastrotatemag = false; double lastEncoder = 0; double modifiedCurrentPos;

    public double RotateMethod(double RightTrig, double Lefttrig, double rotateEncoder, boolean rotateMagnet){

        //sets the target position of the rotate turret using the triggers
       if(RightTrig > .05){
           rotateSet = rotateSet + 16 * RightTrig;
       }else if(Lefttrig > .05){
           rotateSet = rotateSet + 16 * Lefttrig;
       }



        //Uses a magnetic sensor on the turret to reset where the turret is
        // we do this to get the most accurate current position possible
        if(rotateMagnet == true && lastrotatemag == false){
            if(lastEncoder - rotateEncoder < 0){
                modifiedCurrentPos = 20;
            }else if(lastEncoder - rotateEncoder > 0){
                modifiedCurrentPos = -20;
            }
            lastrotatemag = true;
        }else if(rotateMagnet == true){
            lastrotatemag = true;
        }else if(rotateMagnet == false){
            lastrotatemag = false;
        }

        //Hpivot Limits
        if(rotateSet < rotateMin){
            rotateSet = rotateMin;
        }else if(rotateSet > rotateMax){
            rotateSet = rotateMax;
        }

        //PD cycle to control the position of the turret
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

        //Uses a magnetic sensor on the turret to reset where the turret is
        // we do this to get the most accurate current position possible
        if(rotateMagnet == true && lastrotatemag == false){
            if(lastEncoder - rotateEncoder < 0){
                modifiedCurrentPos = 20;
            }else if(lastEncoder - rotateEncoder > 0){
                modifiedCurrentPos = -20;
            }
            lastrotatemag = true;
        }else if(rotateMagnet == true){
            lastrotatemag = true;
        }else if(rotateMagnet == false){
            lastrotatemag = false;
        }

        //Hpivot Limits
        if(rotateSet < rotateMin){
            rotateSet = rotateMin;
        }else if(rotateSet > rotateMax){
            rotateSet = rotateMax;
        }

//PD cycle to control the position of the turret
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
