
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="CL_Teleop_arm_FF", group="Linear Opmode")
public class CL_Teleop_Arm_FF extends LinearOpMode {

    // Declare OpMode members.
    BNO055IMU imu;
    Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;

    @Override
    public void runOpMode() {

        double KpArm = .02;
        double KiArm = .00006;
        double KdArm = .04;
        double KfArm = -0.25;
        double armGoal = 0;
        double ticksPerDeg = 6.25;

        PID arm_PID = new PID(KpArm, KiArm, KdArm);

        //HardwareDevice.Manufacturer;
        arm = hardwareMap.get(DcMotor.class, "arm_motor");

        //Set default direction
        arm.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        arm.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double pitch = getPitch();
            double yeet = armGoal + pitch;

            //arm Code
            double armAngle = arm.getCurrentPosition()/ticksPerDeg;

            double armPower = arm_PID.runPID(yeet, armAngle) + KfArm * Math.sin((armAngle - pitch) * 0.0174533);

            if(gamepad1.start){
                arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
                armGoal = 0;
            }
            if (!gamepad1.right_stick_button) {
                arm.setPower(armPower);
            } else {
                arm.setPower(0);
            }

            if(gamepad1.y){
                armGoal = -90;
            }
            if(gamepad1.x){
                armGoal = 0;
            }
            if(gamepad1.b){
                armGoal = 90;
            }

            armGoal = armGoal + (gamepad1.right_stick_y * .5);

            // Show the elapsed game time and wheel power.
            telemetry.addData("stuff", " armPower (%.2f), armEncoder (%.2f),pitch (%.2f),", armPower, armAngle,pitch);
            telemetry.update();
        }
    }
    public double getPitch() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }


}
