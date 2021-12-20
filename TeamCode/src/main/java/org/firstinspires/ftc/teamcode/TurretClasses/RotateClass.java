package org.firstinspires.ftc.teamcode.TurretClasses;


public class RotateClass{
//setting variables for use later in code
    double rotateMin = -2500, rotateMax = 2500, rotateSet = 0;
    double rotateDifference = 0, MultipliedP = 0, rotateP = -.01, rotateD = 0; double lastError = 0; double MultipliedD = 0; public double rotateMotorPower;
    boolean lastrotatemag = false; double lastEncoder = 0; double modifiedCurrentPos;

    boolean hasMagPrev = false, isRotateHomed = false;
    double homingNextSet = 0, homingFinal = 0, homingmotorpower = 0, deltaEncoder = 0;

    // main code for rotate for use in TeleOp
    public double RotateMethod(double RightTrig, double Lefttrig, double rotateEncoder, boolean rotateMagnet){
        //sets the target position of the rotate turret using the triggers
       if(RightTrig > .05){
           rotateSet = rotateSet + 35 * RightTrig;
       }else if(Lefttrig > .05){
           rotateSet = rotateSet + -35 * Lefttrig;
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

        return rotateMotorPower;
    }

    //main code for rotate in autonomous
    public double RotateAutoMethod(double desiredset, double speed, double rotateEncoder, boolean rotateMagnet){
        rotateSet = desiredset;//using the input to the method to set a universal set point variable

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
