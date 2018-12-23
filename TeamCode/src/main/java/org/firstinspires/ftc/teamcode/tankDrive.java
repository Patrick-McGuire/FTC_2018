package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class tankDrive {
    private DcMotor leftMotor, rightMotor = null;

    public tankDrive(DcMotor leftMotor, DcMotor rightMotor){

    }

    public void driveSteering(double power, double steering) {
        double leftPower = power + steering;
        double rightPower = power - steering;

        driveTank(rightPower, leftPower);
    }

    public void driveTank(double rightPower, double leftPower){
        leftMotor.setPower(Range.clip((leftPower), -1, 1));
        rightMotor.setPower(Range.clip((rightPower), -1, 1));
    }
}
