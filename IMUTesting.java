package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "IMUTesting", group = "Linear")
public class IMUTesting extends LinearOpMode {

    private DcMotor motor;
 	private BNO055IMU imu;

 	double globalAngle;

	
    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.dcMotor.get("motor");
        

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()){
        	getAngle();
        	if(globalAngle > 0){
        		motor.setPower(0.5);
        	} else {
        		motor.setPower(-0.5);
        	}
        	telemetry.addData("angle", globalAngle);
            telemetry.update();
        }

        
    }

    private void getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = angles.firstAngle;


        //double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //if (deltaAngle < -180)
        //    deltaAngle += 360;
        //else if (deltaAngle > 180)
        //    deltaAngle -= 360;

        //globalAngle += deltaAngle;

        //lastAngles = angles;

        //return globalAngle;
    }

	private void timer(long time) throws InterruptedException {
        Thread.sleep(time);
    }
}
