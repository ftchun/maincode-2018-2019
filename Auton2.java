package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auton2", group = "Linear")
public class Auton2 extends LinearOpMode {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
	private Servo servo;
	
    @Override
    public void runOpMode() throws InterruptedException {

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
		servo = hardwareMap.servo.get("servo");
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();


        //timer(1000);

		move(-6000,-.5, 4000);

        servo.setPosition(.1);
		timer(2000);

		move(1000, .5, 2000);
		servo.setPosition(1);
		timer(1000);
    }

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

	private void timer(long time) throws InterruptedException {
        Thread.sleep(time);
    }
}
