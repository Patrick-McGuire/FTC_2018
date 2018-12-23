package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PID {

    public double Kp, Ki, Kd;
    public double error, sum, lastTime = 0;
    public ElapsedTime runtime;

    public PID(double Kp, double Ki, double Kd){
        runtime = new ElapsedTime();
    }

    public double runPID(double target, double input) {
        double oldError = error;
        error = target - input;

        double deltaTime = runtime.milliseconds() - lastTime;
        lastTime = runtime.milliseconds();

        //PID Math
        //P math
        double p = Kp * error;

        if (Math.abs(error) < 15) {
            //I math
             sum += error * deltaTime;
        } else {
            sum = 0;
        }
        double i = Ki * sum;
        i = Range.clip(i, -.1, .1);

        //D math
        double d = Kd * ((error - oldError) / deltaTime);

        //PID calculations
        return (p + i + d);
    }
}
