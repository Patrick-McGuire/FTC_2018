package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Arm {
    double Kp = .02;
    double Ki = .00006;
    double Kd = .04;
    double Kf = -0.25;
    double zeoPos = -125;
    boolean enable = true;

    private DcMotor armMotor;
    public PID arm_PID = new PID(Kp, Ki, Kd);;

    public Arm(DcMotor arm){
        armMotor = arm;
    }

    public void goToPos(double goal, double pitch) {

        if (pitch > 30){
            pitch = -20;
        } else if (pitch < -30){
            pitch = 20;
        }

        double Angle = (armMotor.getCurrentPosition() / 6.25) + zeoPos;
        double armPwr = arm_PID.runPID(goal + pitch, Angle) + feedF(Angle, pitch);

        driveArm(armPwr);
    }

    private double feedF(double angle, double pitch){
        return Kf * Math.sin((angle - pitch) * 0.0174533);
    }

    private void driveArm(double armPower){
        if (enable){
            armMotor.setPower(armPower);
        }
    }
    public void enable(){
        enable = true;
    }

    public void disable(){
        enable = false;
        armMotor.setPower(0);
    }
    public double getZeroPos(){
        return zeoPos;
    }
}
