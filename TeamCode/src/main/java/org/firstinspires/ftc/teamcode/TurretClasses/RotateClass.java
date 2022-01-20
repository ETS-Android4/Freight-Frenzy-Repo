package org.firstinspires.ftc.teamcode.TurretClasses;


public class RotateClass{
    //setting variables for use later in code
    double rotateMin = -4500, rotateMax = 4500, rotateSet = 0;
    double rotateDifference = 0, MultipliedP = 0, rotateP = -.01, rotateD = 0, rotateI = 0; double lastError = 0; double MultipliedD = 0; public double rotateMotorPower;
    boolean lastrotatemag = false; double lastEncoder = 0; double modifiedCurrentPos;

    boolean hasMagPrev = false, isRotateHomed = false;
    double homingNextSet = 0, homingFinal = 0, homingmotorpower = 0, deltaEncoder = 0;

    public double currentSpeed = 0, tickperdegreeRotate = 20, rotateDirection, rotateSpeedSet, speedDifference;
    double integralSum, integralMax = 10, lastSpeedDifference;


    public double RotateSpeedMethod(double desiredset, double speed, double rotateEncoder, boolean rotateMagnet, double time){
        rotateSet = desiredset;//using the input to the method to set a universal set point variable

        rotateSpeedSet = speed;

        if(rotateSet > rotateMax){
            rotateSet = rotateMax;
        }else if(rotateSet < rotateMin){
            rotateSet = rotateMin;
        }

        //setting the modified current position and finding how much we moved since the last loop cycle
        deltaEncoder = rotateEncoder - lastEncoder;

        modifiedCurrentPos = modifiedCurrentPos + deltaEncoder;

        currentSpeed = (deltaEncoder/tickperdegreeRotate)/time;

        if(rotateSet < modifiedCurrentPos){
            rotateDirection = -1;
        }else if(rotateSet > modifiedCurrentPos){
            rotateDirection = 1;
        }

        rotateSpeedSet = Math.copySign(rotateSpeedSet, rotateDirection);

        if((modifiedCurrentPos/tickperdegreeRotate) < 15){
            rotateSpeedSet = rotateSpeedSet * ((modifiedCurrentPos/tickperdegreeRotate)/15);
        }

        speedDifference = rotateSpeedSet - currentSpeed;

        integralSum = integralSum + (speedDifference * time);

        if (integralSum > integralMax){
            integralSum = integralMax;
        }else if (integralSum < -integralMax){
            integralSum = -integralMax;
        }

        rotateMotorPower = (speedDifference * rotateP) + (integralSum * rotateI) + ((speedDifference - lastSpeedDifference) * rotateD);

        lastEncoder = rotateEncoder;
        lastSpeedDifference = speedDifference;

        return rotateMotorPower;
    }



    //main code for rotate in autonomous
    public double RotateAutoMethod(double desiredset, double speed, double rotateEncoder, boolean rotateMagnet){
        rotateSet = desiredset;//using the input to the method to set a universal set point variable

        deltaEncoder = rotateEncoder - lastEncoder;

        modifiedCurrentPos = modifiedCurrentPos + deltaEncoder;

        //Hpivot Limits
        if(rotateSet < rotateMin){
            rotateSet = rotateMin;
        }else if(rotateSet > rotateMax){
            rotateSet = rotateMax;
        }

        //PD cycle to control the position of the turret
        rotateDifference = modifiedCurrentPos - rotateSet;
        MultipliedP = rotateDifference * rotateP;
        MultipliedD = (rotateDifference - lastError) * rotateD;
        rotateMotorPower = MultipliedP + MultipliedD;


        lastError = rotateDifference;
        lastEncoder = rotateEncoder;
        //setting limits for speed to go a desired setpoint
        if(rotateMotorPower > speed){
            rotateMotorPower = speed;
        }else if(rotateMotorPower < -speed){
            rotateMotorPower = -speed;
        }
        return rotateMotorPower;
    }

    public double RotateHoming(boolean RotateMagnetic, double rotateEncoder){

        if(RotateMagnetic == false){
            if(hasMagPrev == false){
                homingNextSet = homingNextSet + 10;
                homingmotorpower = RotateAutoMethod(homingNextSet, .5, rotateEncoder, RotateMagnetic);
            }else{
                isRotateHomed = true;
                homingFinal = rotateEncoder;
                modifiedCurrentPos = 20;
            }

        }else{
            homingNextSet = homingNextSet - 10;
            homingmotorpower =  RotateAutoMethod(homingNextSet, .5, rotateEncoder, RotateMagnetic);
            hasMagPrev = true;
        }
        return homingmotorpower;
    }
    public boolean isHomedRotateReturn(){return isRotateHomed;}
    public double modifiedRotateCurrent(){return modifiedCurrentPos;}
    public double rotatemotorPowerReturn(){return rotateMotorPower;}

}
