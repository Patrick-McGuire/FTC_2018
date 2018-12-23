
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="fghfkj", group="Linear Opmode")
public class PID_Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;

    double Kp = 20;
    double Ki = 0;
    double Kd = 0;

    double p;
    double i;
    double d;

    double error;
    double goal;

    double deltaTime;
    double lastTime = 0;
    double sum = 0;

    double outputa;

    @Override
    public void runOpMode() {

        arm = hardwareMap.get(DcMotor.class, "arm_motor");
        arm.setDirection(DcMotor.Direction.FORWARD);

        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {

            //1120 tics per rev for 40:1
            goal += (gamepad1.left_stick_y*10);
            double input = 360*(arm.getCurrentPosition()/1120);
            double oldError = error;
            error = goal - input;

            deltaTime = runtime.milliseconds() - lastTime;
            lastTime = runtime.milliseconds();

            //PID Math
            //P math
            p= Kp*error;

            //I math
            sum += error * deltaTime;
            i += Ki * sum;

            //D math
            d = Kd * ((error - oldError)/deltaTime);

            //PID calculations
            outputa = (p+i+d);

            arm.setPower(Range.clip((outputa/8000),-.8,.8));

            if (gamepad1.a) {
                goal = 300;
            }
            if (gamepad1.b) {
                goal = 1000;
            }
            if (gamepad1.y) {
                goal = 1800;
            }
            if (gamepad1.x) {
                arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
                arm.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
                goal = 0;
            }
            telemetry.addData("stuff", "output (%.2f), input (%.2f), motor output (%.2f), Kp (%.2f, Goal (%.2f)", outputa, input, outputa/8000, Kp,goal);
            telemetry.update();
        }
    }
}