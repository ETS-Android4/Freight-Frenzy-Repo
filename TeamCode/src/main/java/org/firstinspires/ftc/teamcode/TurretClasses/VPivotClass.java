package org.firstinspires.ftc.teamcode.TurretClasses;


import org.firstinspires.ftc.teamcode.TestHub.FreightFrenzyHardwareMap;

public class VPivotClass extends FreightFrenzyHardwareMap{
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    //setting variables for use in later code
    public double vPivotSet = 1.15;
    public double FinalMotorPower;
    public double lastTime, lastPOT, speed, speedDifference, speedCorrection, speedPM, speedDM, lastSpeedDifference = 0, vPivotPOT1degree = .0105, vPivotEncoder1Degree = 23;//TODO correct this variable
    public double degreesTraveled, speedSetPoint;
    public double vPivotCorrection = 1, encoderWithOffset = 0, deltaPivotEncoder, lastEncoder = 0, motionprofile = 0, lastSpeed = 0;
    public boolean has1stloop = false;
    public  double UPSpeedPM = .015, UPSpeedDM = .009, DNSpeedPM = .004, DNSpeedDM = .012, closeSpeedMult;
    double StartSlowSpeed = .25, time = robot.TimerCustom();



    public double VPivot(double SetPoint, double SpeedSetPt , double EncoderReading, boolean MAG){



        //setting our encoder offset using a potentiometer for the starting number
        //so we get an accurate start pos because the encoder resets every time we start the program

        if(MAG == false && has1stloop == false) {
            encoderWithOffset = 2100;//TODO can we put this in init?
            has1stloop = true;
            StartSlowSpeed = 1;

        }

        //setting set point limits on the arm to prevent the arm from over rotating
        if(SetPoint > 3000){//minimum travel
            vPivotSet = 3000;
        }else if (SetPoint < 0){//maximum travel
            vPivotSet = 0;
        }else{
            vPivotSet = SetPoint;
        }

        //calculating the current position based off the change in encoder and the offset set at the start of the program
        deltaPivotEncoder = (EncoderReading) - lastEncoder;
        encoderWithOffset = encoderWithOffset + deltaPivotEncoder;


        //Calculating Speed at tip of arm
        degreesTraveled = deltaPivotEncoder / vPivotEncoder1Degree;
        speed = ((2 * 16) * Math.PI)*(degreesTraveled / 360) / (time - lastTime);


        //Motion Profile
        if(Math.abs(vPivotSet - encoderWithOffset) < 300){
            speedSetPoint = SpeedSetPt * ((Math.abs(vPivotSet - encoderWithOffset))/300);
        }else {
            speedSetPoint = SpeedSetPt;
        }

        //setting a deadzone to prevent small oscillations at the setpoint
        if (Math.abs(vPivotSet - encoderWithOffset) < 50){
            closeSpeedMult = .2;
            speedSetPoint = 0;
        }else{
            closeSpeedMult = 1;
        }

        //Vertical Pivot direction setting
        //we look to see if we are lower or higher than the setpoint and set a positive or negative value to determine which way the arm has to pivot
        //we have 2 different Proportional and Derivative multipliers for different correction directions
        if(vPivotSet > encoderWithOffset){
            vPivotCorrection = 1;
            speedPM = UPSpeedPM;
            speedDM = UPSpeedDM;
        }else if(vPivotSet < encoderWithOffset){
            vPivotCorrection = -1;
            if(speedSetPoint < speed){
                speedPM = UPSpeedPM;
                speedDM = UPSpeedDM;
            }else{
                speedPM = DNSpeedPM;
                speedDM = DNSpeedDM;
            }
        }

        speedSetPoint = Math.copySign(speedSetPoint, vPivotCorrection);

        //Speed Proportional and derivative correction equations to make the arm move at a desired speed not motor power
        speedDifference = speedSetPoint - speed;
        speedCorrection = (speedDifference * speedPM) + ((speedDifference - lastSpeedDifference) * speedDM);


        //combining the speed correction and the direction pos/neg sign for the correct direction
        FinalMotorPower = FinalMotorPower + (speedCorrection * closeSpeedMult * StartSlowSpeed);

        //limiting the motor power to 1 which is the maximum the motor can go
        if(FinalMotorPower > 1){
            FinalMotorPower = 1;
        }else if(FinalMotorPower < -1){
            FinalMotorPower = -1;
        }

        //setting variables for use in the next loop cycle
        lastTime = time;
        lastSpeedDifference = speedDifference;
        lastEncoder = EncoderReading;
        lastSpeed = speed;

        return FinalMotorPower;
    }


    //Mew method for better control of the turret
    public double NEWVPivot(double SetPoint, double SpeedSetPt ,double POTReading, double EncoderReading, boolean MAG , double time,
                            double extendlength, double UPPM, double UPDM, double DNPM, double DNDM, double MinSpeed){

        //for use of the FTC Dashboard ONLY,
        //TODO get rid of once done tuning
        UPSpeedPM = UPPM;
        UPSpeedDM = UPDM;

        DNSpeedPM = DNPM;
        DNSpeedDM = DNDM;

        //setting our encoder offset using a potentiometer for the starting number
        //so we get an accurate start pos because the encoder resets every time we start the program

        if(MAG == false && has1stloop == false) {
            encoderWithOffset = 2100;//TODO can we put this in init?
            has1stloop = true;
            StartSlowSpeed = 1;

        }

        //setting set point limits on the arm to prevent the arm from over rotating
        if(SetPoint > 3000){//minimum travel
            vPivotSet = 3000;
        }else if (SetPoint < 0){//maximum travel
            vPivotSet = 0;
        }else{
            vPivotSet = SetPoint;
        }

        //calculating the current position based off the change in encoder and the offset set at the start of the program
        deltaPivotEncoder = (EncoderReading) - lastEncoder;
        encoderWithOffset = encoderWithOffset + deltaPivotEncoder;


        //Calculating Speed at tip of arm
        degreesTraveled = deltaPivotEncoder / vPivotEncoder1Degree;
        speed = ((2 * extendlength) * Math.PI)*(degreesTraveled / 360) / (time - lastTime);


        //Motion Profile
        if(Math.abs(vPivotSet - encoderWithOffset) < 300){
            speedSetPoint = SpeedSetPt * ((Math.abs(vPivotSet - encoderWithOffset))/300);
        }else {
            speedSetPoint = SpeedSetPt;
        }

        //setting a deadzone to prevent small oscillations at the setpoint
        if (Math.abs(vPivotSet - encoderWithOffset) < 50){
            closeSpeedMult = MinSpeed;//TODO set this as the dashboard #
            speedSetPoint = 0;
        }else{
            closeSpeedMult = 1;
        }

        //Vertical Pivot direction setting
        //we look to see if we are lower or higher than the setpoint and set a positive or negative value to determine which way the arm has to pivot
        //we have 2 different Proportional and Derivative multipliers for different correction directions
        if(vPivotSet > encoderWithOffset){
            vPivotCorrection = 1;
            speedPM = UPSpeedPM;
            speedDM = UPSpeedDM;
        }else if(vPivotSet < encoderWithOffset){
            vPivotCorrection = -1;
            if(speedSetPoint < speed){
                speedPM = UPSpeedPM;
                speedDM = UPSpeedDM;
            }else{
                speedPM = DNSpeedPM;
                speedDM = DNSpeedDM;
            }
        }

        speedSetPoint = Math.copySign(speedSetPoint, vPivotCorrection);

        //Speed Proportional and derivative correction equations to make the arm move at a desired speed not motor power
        speedDifference = speedSetPoint - speed;
        speedCorrection = (speedDifference * speedPM) + ((speedDifference - lastSpeedDifference) * speedDM);


        //combining the speed correction and the direction pos/neg sign for the correct direction
        FinalMotorPower = FinalMotorPower + (speedCorrection * closeSpeedMult * StartSlowSpeed);

        //limiting the motor power to 1 which is the maximum the motor can go
        if(FinalMotorPower > 1){
            FinalMotorPower = 1;
        }else if(FinalMotorPower < -1){
            FinalMotorPower = -1;
        }

        //setting variables for use in the next loop cycle
        lastTime = time;
        lastPOT = POTReading;
        lastSpeedDifference = speedDifference;
        lastEncoder = EncoderReading;
        lastSpeed = speed;

        return FinalMotorPower;
    }

    public double PivotSetReturn(){return vPivotSet; }
    public double SpeedReturn(){return  speed;}
    public double DegreesTravelReturn(){ return degreesTraveled;}
    public double LastPOTreadingReturn(){return lastPOT;}
    public double MotionProfle(){return motionprofile; }
}
