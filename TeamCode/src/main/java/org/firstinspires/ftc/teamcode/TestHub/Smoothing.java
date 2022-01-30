package org.firstinspires.ftc.teamcode.TestHub;

public class Smoothing {
    double Gpad1xinput1 = 0,Gpad1xinput2 = 0,Gpad1xinput3 = 0,Gpad1xinput4 = 0,Gpad1xinput5 = 0,Gpad1xinput6 = 0,Gpad1xinput7 = 0,Gpad1xinput8 = 0,Gpad1xinput9 = 0,Gpad1xinput10 = 0;
    public double Gpad1XSmooth (double input){
        Gpad1xinput10 = Gpad1xinput9;
        Gpad1xinput9 = Gpad1xinput8;
        Gpad1xinput8 = Gpad1xinput7;
        Gpad1xinput7 = Gpad1xinput6;
        Gpad1xinput6 = Gpad1xinput5;
        Gpad1xinput5 = Gpad1xinput4;
        Gpad1xinput2 = Gpad1xinput3;
        Gpad1xinput3 = Gpad1xinput2;
        Gpad1xinput2 = Gpad1xinput1;
        Gpad1xinput1 = input;


        return (Gpad1xinput1 + Gpad1xinput2 + Gpad1xinput3 + Gpad1xinput4 + Gpad1xinput5 + Gpad1xinput6 + Gpad1xinput7 + Gpad1xinput8 + Gpad1xinput9 + Gpad1xinput10)/10;


    }

    double TestTotal;
    int TestArrayNum = 0, TestfirstLoop = 0;
    double TestArray[] = new double[10];

    public double SmoothTest(double input){
        if(TestfirstLoop == 0) {
            TestfirstLoop = 1;
            for (int arrayInitialSet = 0; arrayInitialSet < 10; arrayInitialSet++) {
                TestArray[arrayInitialSet] = 0;
            }
        }
        TestTotal = TestTotal - TestArray[TestArrayNum];

        TestArray[TestArrayNum] = input;

        TestTotal = TestTotal + TestArray[TestArrayNum];


        TestArrayNum = TestArrayNum + 1;
        if(TestArrayNum >= 10){
            TestArrayNum = 0;
        }

        if(Math.abs(input) < .08){
            for (int arrayInitialSet = 0; arrayInitialSet < 10; arrayInitialSet++) {
                TestArray[arrayInitialSet] = 0;
            }
            TestTotal = 0;
        }

        return TestTotal/10;

    }

    double DriveYTotal;
    int DriveYArrayNum = 0, DriveYfirstLoop = 0;
    double DriveYArray[] = new double[10];

    public double SmoothDriveY(double input){
        if(DriveYfirstLoop == 0) {
            DriveYfirstLoop = 1;
            for (int DriveYInitialSet = 0; DriveYInitialSet < 10; DriveYInitialSet++) {
                DriveYArray[DriveYInitialSet] = 0;
            }
        }
        DriveYTotal = DriveYTotal - DriveYArray[DriveYArrayNum];

        DriveYArray[DriveYArrayNum] = input;

        DriveYTotal = DriveYTotal + DriveYArray[DriveYArrayNum];


        DriveYArrayNum = DriveYArrayNum + 1;
        if(DriveYArrayNum >= 10){
            DriveYArrayNum = 0;
        }

        if(Math.abs(input) < .08){
            for (int DriveYInitialSet = 0; DriveYInitialSet < 10; DriveYInitialSet++) {
                DriveYArray[DriveYInitialSet] = 0;
            }
            DriveYTotal = 0;
        }

        return DriveYTotal/10;

    }

    double DriveXTotal;
    int DriveXArrayNum = 0, DriveXfirstLoop = 0;
    double DriveXArray[] = new double[10];

    public double SmoothDriveX(double input){
        if(DriveXfirstLoop == 0) {
            DriveXfirstLoop = 1;
            for (int DriveXInitialSet = 0; DriveXInitialSet < 10; DriveXInitialSet++) {
                DriveXArray[DriveXInitialSet] = 0;
            }
        }
        DriveXTotal = DriveXTotal - DriveXArray[DriveXArrayNum];

        DriveXArray[DriveXArrayNum] = input;

        DriveXTotal = DriveXTotal + DriveXArray[DriveXArrayNum];


        DriveXArrayNum = DriveXArrayNum + 1;
        if(DriveXArrayNum >= 10){
            DriveXArrayNum = 0;
        }

        if(Math.abs(input) < .08){
            for (int DriveXInitialSet = 0; DriveXInitialSet < 10; DriveXInitialSet++) {
                DriveXArray[DriveXInitialSet] = 0;
            }
            DriveXTotal = 0;
        }

        return DriveXTotal/10;

    }

    double DriveZTotal;
    int DriveZArrayNum = 0, DriveZfirstLoop = 0;
    double DriveZArray[] = new double[10];

    public double SmoothDriveZ(double input){
        if(DriveZfirstLoop == 0) {
            DriveZfirstLoop = 1;
            for (int DriveZInitialSet = 0; DriveZInitialSet < 10; DriveZInitialSet++) {
                DriveZArray[DriveZInitialSet] = 0;
            }
        }
        DriveZTotal = DriveZTotal - DriveZArray[DriveZArrayNum];

        DriveZArray[DriveZArrayNum] = input;

        DriveZTotal = DriveZTotal + DriveZArray[DriveZArrayNum];


        DriveZArrayNum = DriveZArrayNum + 1;
        if(DriveZArrayNum >= 10){
            DriveZArrayNum = 0;
        }

        if(Math.abs(input) < .08){
            for (int DriveZInitialSet = 0; DriveZInitialSet < 10; DriveZInitialSet++) {
                DriveZArray[DriveZInitialSet] = 0;
            }
            DriveZTotal = 0;
        }

        return DriveZTotal/10;

    }
}
