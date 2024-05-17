#ifndef PAIRING_ROBOT_HH
#define PAIRING_ROBOT_HH

#include "robot.hh"
#include <memory>
#include <vector>

class PairingRobot;
typedef std::shared_ptr<PairingRobot> PairingRobotPtr;

class PairingRobot : public mrs::Robot {
public:
    PairingRobot(unsigned int id, const mrs::Position2d & p, 
                 const mrs::RobotSettings & settings = mrs::defaultRobotSettings,
                 const mrs::Velocity2d & vel = mrs::Velocity2d::Random());

    const mrs::Velocity2d & action(std::vector<mrs::RobotPtr> & swarm) override;
    std::string name() const override { return std::string("Pair"); }
    mrs::RobotPtr clone() const override;

private:
    enum State { WANDERING, COORDINATED_MOVEMENT, REPULSION };

    State state;
    int timer;
    mrs::Position2d partnerPosition;
    float partnerColor[3];
    float coordinatedAngle;
    float desiredDistance;

    void wander(std::vector<mrs::RobotPtr> & swarm);
    void coordinatedMovement(std::vector<mrs::RobotPtr> & swarm);
    void repulsion(std::vector<mrs::RobotPtr> & swarm);
};

#endif // PAIRING_ROBOT_HH
