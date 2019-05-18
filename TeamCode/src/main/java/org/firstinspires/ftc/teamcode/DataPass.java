package org.firstinspires.ftc.teamcode;

public class DataPass {
    //#0 arm goal
    //#1 distance
    //#2 angle
    int[] Goals = new int[3];

    public void setGoals(int[] goals){
        Goals = goals;
    }
    public int[] getGoals(){
        return Goals;
    }



}
