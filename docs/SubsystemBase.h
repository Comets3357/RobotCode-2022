// where robot init at

#pragma once

namespace Comets3357
{
    class SubsystemBase
    {
    public:
        SubsystemBase();

        /**
     * Periodic code for all modes should go here.
     */
        virtual void RobotInit() {} 

        /**
     * Periodic code for all modes should go here.
     */
        virtual void RobotPeriodic() {}
      
        /**
     * Initialization code for disabled mode should go here.
     */
        virtual void DisabledInit() {}

        /**
     * Periodic code for disabled mode should go here.
     */
        virtual void DisabledPeriodic() {}
        /**
     * Initialization code for autonomous mode should go here.
     */
        virtual void AutonomousInit() {}

        /**
     * Periodic code for autonomous mode should go here.
     */
        virtual void AutonomousPeriodic() {}

        /**
     * Initialization code for teleop mode should go here.
     */
        virtual void TeleopInit() {}

        /**
     * Periodic code for teleop mode should go here.
     */
        virtual void TeleopPeriodic() {}

        /**
     * Initialization code for test mode should go here.
     */
        virtual void TestInit() {}
      
        /**
     * Periodic code for test mode should go here.
     */
        virtual void TestPeriodic() {}

    private:
    };
} // namespace Comets3357


