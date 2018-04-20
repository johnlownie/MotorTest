#ifndef _ROBOT_HG_
#define _ROBOT_HG_

#include <WPILib.h>
#include <SampleRobot.h>
#include <ctre/Phoenix.h>
#include <XboxController.h>

/**
 *
 */
class Robot : public frc::SampleRobot
{
public:
	Robot();
	~Robot();

	void RobotInit() override;
	void Autonomous() override;
	void OperatorControl() override;
	void Test() override;

private:
	void   SetMotor(int motor_id);
	void   Trace(void);

	int    iMotorId;
	int    iCounter;
	double dMotorSpeed;
	bool   bIsInverted;
	bool   bIsPhased;
	bool   bIsClosedMode;

	WPI_TalonSRX* pTalonSRX;
	Faults* pFaults;

	frc::XboxController* pXboxController;
};

#endif
