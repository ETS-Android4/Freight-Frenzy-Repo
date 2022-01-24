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

        return TestTotal/10;

    }
}
