package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.SynchronousQueue;

public class Thread1 extends Thread{
    //#0 arm goal
    //#1 distance
    //#2 angle
    int[] Goals = new int[3];
    int[] Inputs;
    private ElapsedTime runtime = new ElapsedTime();
    public DataPass DataPassz;

    public Thread1(DataPass DataPasszz){
        DataPassz = DataPasszz;
    }

    public void run(){

      //setGoals(400,500,0);
      //waitDist(20, .2);

      setGoals(400,0,180);
      //waitAngle(5,1);

      //setGoals(400,0,-180);
      //waitAngle(5,1);

      //setGoals(400,0,0);
    }

    public void setGoals(int arm, int distance, int angle){
        Goals[0] = arm;
        Goals[1] = distance;
        Goals[2] = angle;
        DataPassz.setGoals(Goals);
    }

    public void waitDist(int allowedError, double timeOut){
        Inputs = DataPassz.getInputs();
        while (!distanceInBounds(Goals[1] - Inputs[1],allowedError) ){
            Inputs = DataPassz.getInputs();
        }

        double time = runtime.seconds();
        while (runtime.seconds() < time + timeOut){
            Inputs = DataPassz.getInputs();
        }
    }
    public void waitAngle(int allowedError, double timeOut){
        Inputs = DataPassz.getInputs();
        while (!distanceInBounds(Goals[2] - Inputs[2],allowedError) ){
            Inputs = DataPassz.getInputs();
        }

        double time = runtime.seconds();
        while (runtime.seconds() < time + timeOut){
            Inputs = DataPassz.getInputs();
        }
    }


    public boolean distanceInBounds(int error, int allowedError){
        if (Math.abs(error) < allowedError){
            return true;
        }
        return false;
    }
    public boolean angleInBounds(int error, int allowedError){
        if (Math.abs(error) < allowedError){
            return true;
        }
        return false;
    }
}
