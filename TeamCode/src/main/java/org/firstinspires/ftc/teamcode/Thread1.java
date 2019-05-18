package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.SynchronousQueue;

public class Thread1 extends Thread{
    //#0 arm goal
    //#1 distance
    //#2 angle
    int[] Goals = new int[3];
    int i = 0;
    private ElapsedTime runtime = new ElapsedTime();
    public DataPass DataPassz;
    //public SynchronousQueue<Thread1> synQue = new SynchronousQueue<>();

    public Thread1(DataPass DataPasszz){
        DataPassz = DataPasszz;
    }

    public void run(){

        Goals[0] = 400;
        Goals[1] = 500;
        Goals[2] = 0;
        DataPassz.setGoals(Goals);

        while ( runtime.seconds() < 5){
            i = 0;
        }

        Goals[0] = 800;
        Goals[1] = 500;
        Goals[2] = 0;
        DataPassz.setGoals(Goals);

        while ( runtime.seconds() < 10){
            i = 0;
        }

        Goals[0] = 400;
        Goals[1] = 0;
        Goals[2] = 0;
        DataPassz.setGoals(Goals);
    }

}
