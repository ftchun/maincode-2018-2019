package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Maybe First", group = "Linear")
public class MaybeFirst extends LinearOpMode {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;



    @Override
    public void runOpMode() throws InterruptedException {

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int speedConstant = 1;
        boolean prevX = false;
        boolean prevY = false;

        waitForStart();

        while (opModeIsActive()) {
            if(prevY == false && gamepad1.y && speedConstant>0)
            {
                speedConstant -= 1;
            }
            if(prevX == false && gamepad1.x && speedConstant<2)
            {
                speedConstant += 1;
            }

            prevX = gamepad1.x;
            prevY = gamepad1.y;

            if(speedConstant == 0)
            {
                motorBL.setPower(-gamepad1.left_stick_y/2);
                motorFL.setPower(-gamepad1.left_stick_y/2);
                motorBR.setPower(-gamepad1.right_stick_y/2);
                motorFR.setPower(-gamepad1.right_stick_y/2);
            }

            if(speedConstant == 1)
            {
                motorBL.setPower(-gamepad1.left_stick_y/4);
                motorFL.setPower(-gamepad1.left_stick_y/4);
                motorBR.setPower(-gamepad1.right_stick_y/4);
                motorFR.setPower(-gamepad1.right_stick_y/4);
            }

            if(speedConstant == 2)
            {
                motorBL.setPower(-gamepad1.left_stick_y/8);
                motorFL.setPower(-gamepad1.left_stick_y/8);
                motorBR.setPower(-gamepad1.right_stick_y/8);
                motorFR.setPower(-gamepad1.right_stick_y/8);
            }

            int motorBRPosition = motorBR.getCurrentPosition();
            int motorBLPosition = motorBL.getCurrentPosition();

            telemetry.addData("motorBR Position: ", motorBRPosition);
            telemetry.addData("motorBL Position: ", motorBLPosition);
            telemetry.addData("The Speed: ", speedConstant);
            telemetry.update();
        }

        idle();

    }

}
