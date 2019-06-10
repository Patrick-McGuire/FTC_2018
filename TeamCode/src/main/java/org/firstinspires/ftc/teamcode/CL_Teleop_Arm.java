
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="CL_Teleop_arm", group="Linear Opmode")
public class CL_Teleop_Arm extends LinearOpMode {

    // Declare OpMode members.
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMoter = null;
    Orientation angles;
    private DcMotor climber = null;
    private CRServo servo1 = null;
    private CRServo servo2 = null;
    private Arm Arm;

    public double angleOfset = 0;

    @Override
    public void runOpMode() {

        double Kp = .02; //.03;
        double Ki = 0; //.000106;
        double Kd = 0; //8;
        double targetAngle = 0;
        double steering = 0;
        double armGoal = 0;



        PID driveTrain_PID = new PID(Kp, Ki, Kd);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //HardwareDevice.Manufacturer;
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMoter = hardwareMap.get(DcMotor.class, "arm_motor");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        climber = hardwareMap.get(DcMotor.class, "climber_motor");

        //Set default direction
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armMoter.setDirection(DcMotor.Direction.FORWARD);
        climber.setDirection(DcMotor.Direction.FORWARD);

        armMoter.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        armMoter.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        tankDrive driveTrain = new tankDrive(rightDrive, leftDrive);
        Arm = new Arm(armMoter);

        armGoal = Arm.getZeroPos();

        boolean a = true;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double angle = getAngle();

            double st = gamepad1.left_stick_x;

            if (st < 0) {
                steering = ((st * st)) * .7 + .3;
            }
            else if (st > 0) {
                steering = -(st * st) * .7 + -.3;
            } else{
                steering = 0;
            }


            if (steering != 0) {
                runtime.reset();
            }
            if (runtime.milliseconds() < 300) {
                zeroGyro();
                targetAngle = angle;
            } else if (runtime.milliseconds() > 300) {
                steering = driveTrain_PID.runPID(targetAngle, angle);
            }

            double drivePower = gamepad1.right_trigger - gamepad1.left_trigger;
            driveTrain.driveSteering(drivePower, steering);

            if (gamepad1.dpad_left) {
                zeroGyro();
                targetAngle = 90;
            }
            if (gamepad1.dpad_right) {
                zeroGyro();
                targetAngle = -90;
            }

            //intake code
            if (gamepad1.left_bumper) {
                servo1.setPower(1);
                servo2.setPower(-1);
            } else if (gamepad1.left_stick_button) {
                servo1.setPower(-1);
                servo2.setPower(1);
            } else {
                servo1.setPower(0);
                servo2.setPower(0);
            }

            //climber code
            if (gamepad1.dpad_up) {
                climber.setPower(1);
                Arm.disable();
            } else if (gamepad1.dpad_down) {
                climber.setPower(-1);
                Arm.disable();
            } else {
                climber.setPower(0);
                if (gamepad1.right_stick_button && Arm.enable && a){
                    Arm.disable();
                    a = false;
                }
                else if (gamepad1.right_stick_button && !Arm.enable && a){
                    Arm.enable();
                    a = false;
                }
                else if (!gamepad1.right_stick_button){
                     a = true;
                }

            }
            if (gamepad1.right_stick_button){

            }

            //arm code

            double test = (armMoter.getCurrentPosition()/6.25) + Arm.getZeroPos();

            if(gamepad1.y){
                armGoal = 0;
            }
            if(gamepad1.x){
                armGoal = -90;
            }
            if(gamepad1.b){
                armGoal = 90;
            }
            if(gamepad1.a){
                armGoal = 151;
            }

            if(gamepad1.start && gamepad1.left_stick_button){
                armMoter.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
                armGoal = 0;
            }

            Arm.goToPos(armGoal, getPitch());

            double armPower = armMoter.getPower();

            armGoal = armGoal + (gamepad1.right_stick_y * 1.3);
            // Show the elapsed game time and wheel power.
            telemetry.addData("stuff", " armPower (%.2f), armEncoder (%.2f), Kp (%.2f, Goal (%.2f), Kd (%.2f), Ki (%.2f), error (%.2f), pitch (%.2f), pitch (%.2f)", armPower, test, Kp, targetAngle, Kd, Ki * 100000, targetAngle-angle, getPitch(),st);
            telemetry.update();
        }
    }

    public double getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle;

        deltaAngle -= angleOfset;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        return(deltaAngle);
    }

    public void zeroGyro() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleOfset = angles.firstAngle;
    }
    public double getPitch() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }
}
