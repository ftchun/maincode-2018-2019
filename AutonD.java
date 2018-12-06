package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutonD", group = "Linear")
public class AutonD extends LinearOpMode {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
	private Servo servo;

    private DcMotor motorHR;
    private DcMotor motorHL;
	
    @Override
    public void runOpMode() throws InterruptedException {

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

        moveLift(1500, 1500, 1, 6000);

        shiftRight(700, .5, 3000);

        move(-1000, -.5, 2000);

        shiftRight(-500, -.5, 3000);

        move(-5000, -.5, 5000);

        servo.setPosition(.2);
        timer(2000);

        move(1000, .5, 2000);
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

    public void moveLift(int positionL, int positionR, double power, long time) throws InterruptedException {
        motorHL.setTargetPosition(positionL);
        motorHR.setTargetPosition(positionR);
        motorHL.setPower(power);
        motorHR.setPower(power);

        timer(time);
    }

	private void timer(long time) throws InterruptedException {
        Thread.sleep(time);
    }
}
