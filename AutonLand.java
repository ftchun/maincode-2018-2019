package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
*
*/

@Autonomous(name = "Landing only", group = "Linear")
public class AutonLand extends LinearOpMode {

    /*
    * These are the definitions for the motors just like the TeleOp. Four driving motors,
    * a servo, and the hex motors for the lift.
    */

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
	private Servo servo;

    private DcMotor motorHR;
    private DcMotor motorHL;
	
    @Override
    public void runOpMode() throws InterruptedException {

        /*
        * Like before, we are getting the necessary motors from the configuration, using
        * the same naming convention.
        */

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
		servo = hardwareMap.servo.get("servo");
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        motorHL = hardwareMap.dcMotor.get("motorHL");
        motorHR = hardwareMap.dcMotor.get("motorHR");

        motorHL.setDirection(DcMotor.Direction.REVERSE);

        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorHL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorHR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHR.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        waitForStart();


        /*
        *
        */

        moveLift(2400, 2100, 1, 7000);

        shiftRight(3000, .5, 4000);

        move(-2000, -.5, 2000);
    }


    /*
    * This is the main move method. It takes three arguments: distance, power, and time.
    * It sets the back motors to run to position mode, moves them to the target
    * distance and sets the power, sleeps for amount 'time', then sets the powers back
    * to zero and resets the encoders. This allows for simple movement that gets reset
    * every time.
    */

    public void move(int distance, double power, long time) throws InterruptedException {
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setTargetPosition(distance);
        motorBR.setTargetPosition(distance);

        motorBL.setPower(power);
        motorBR.setPower(power);

        timer(time);

        motorBL.setPower(0);
        motorBR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /*
    * The next method is 'shiftRight()', which is essentially a turn right method.
    * It works the same way as 'move()' except it reverses the right wheels in order to
    * turn right. It goes through the same process to stop the motors again.
    */

    public void shiftRight(int distance, double power, long time) throws InterruptedException {
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setTargetPosition(distance);
        motorBR.setTargetPosition(-distance);

        motorBL.setPower(power);
        motorBR.setPower(power);

        timer(time);

        motorBL.setPower(0);
        motorBR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /*
     * Our next method is the 'moveLift()' method. As the title implies, this method is for
     * moving the lift using the hex motors. It simply sets the target position using the
     * two position arguments and sets the power with the power argument. It then sleeps
     * using the 'timer()' method and ends.
     */

    public void moveLift(int positionL, int positionR, double power, long time) throws InterruptedException {
        motorHL.setTargetPosition(positionL);
        motorHR.setTargetPosition(positionR);
        motorHL.setPower(power);
        motorHR.setPower(power);

        timer(time);
    }


    /*
    * The 'timer()' method is simply an easier way to call a sleep in the program, which
    * is necessary to give the motors time to move to their requested positions. It is used
    * in all the above movement methods.
    */

	private void timer(long time) throws InterruptedException {
        Thread.sleep(time);
    }
}
