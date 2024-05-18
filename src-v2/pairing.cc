#include "pairing.hh" // Incluir la cabecera pairing.hh que contiene las declaraciones relacionadas con la clase PairingRobot
#include "environment.hh" // Incluir la cabecera environment.hh para las declaraciones del entorno
#include <cmath> // Incluir la librería cmath para las funciones matemáticas
#include <cstdlib> // Incluir la librería cstdlib para las funciones relacionadas con la generación de números aleatorios

// Constructor de la clase PairingRobot, inicializa los parámetros y configura el estado inicial como WANDERING
PairingRobot::PairingRobot(unsigned int id, const mrs::Position2d & p, 
                           const mrs::RobotSettings & settings,
                           const mrs::Velocity2d & vel)
    : mrs::Robot(id, p, settings), state(WANDERING), timer(0),coordinatedAngle(0.0f),desiredDistance(2.0f),wanderAngle(0.0f){}

// Método clone para crear una copia del robot actual
mrs::RobotPtr PairingRobot::clone() const {
    return std::make_shared<PairingRobot>(*this);
}

// Método action que define el comportamiento del robot según su estado actual
const mrs::Velocity2d & PairingRobot::action(std::vector<mrs::RobotPtr> & swarm) {
    switch (state) {
        case WANDERING:
            wander(swarm); // Si el estado es WANDERING, ejecutar el método wander
            break;
        case COORDINATED_MOVEMENT:
            coordinatedMovement(swarm); // Si el estado es COORDINATED_MOVEMENT, ejecutar el método coordinatedMovement
            break;
        case REPULSION:
            repulsion(swarm); // Si el estado es REPULSION, ejecutar el método repulsion
            break;
    }
    return m_vel; // Retornar la velocidad actual del robot
}
int i = 0;
// Método wander que define el comportamiento del robot en el estado WANDERING
void PairingRobot::wander(std::vector<mrs::RobotPtr> & swarm) {
    std::cout << "Wandering" << std::endl;
    timer++; // Incrementar el contador de tiempo
    if ((timer % 100 == 1) && state==WANDERING) { // Cada 50 unidades de tiempo
        wanderAngle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI; // Generar un ángulo aleatorio
        m_vel = mrs::Velocity2d(0.5f * cos(wanderAngle), 0.5f * sin(wanderAngle)); // Asignar una velocidad constante en una dirección aleatoria
        std::cout << "Robot: " << i << " Wandering" << std::endl;
    }

    // Verificar la cercanía de otros robots en el enjambre
    for (auto& neighbor : swarm) {

        if (!sameColor(neighbor) && mrs::distance(m_pos, neighbor->position()) < 2.0f) {
            // Si se encuentra un robot de diferente color a una distancia menor a 5.0
            partnerPosition = neighbor->position(); // Guardar la posición del compañero
            partnerColor[0] = neighbor->settings().color[0]; // Guardar el color del compañero
            state = COORDINATED_MOVEMENT; // Cambiar el estado a COORDINATED_MOVEMENT
            timer = 0; // Reiniciar el contador de tiempo
            float coordinatedAngle1 =mrs::angle(m_pos, partnerPosition);; // Calcular el ángulo hacia el compañero
            float coordinatedAngle2 =mrs::angle(partnerPosition,m_pos );; // Calcular el ángulo hacia el compañero
            //std::cout << "Robot: " << i << " Coordinated Movement" << std::endl;
            //std::cout << "coordinate: " << coordinatedAngle1 << std::endl;
            //std::cout << "coordinate2: " << coordinatedAngle2 << std::endl;
            coordinatedAngle = mrs::averageAngle({coordinatedAngle1, coordinatedAngle2});
            //std::cout << "coordinate2: " << coordinatedAngle << std::endl;
            i+=1;
            break;
        }
    }
}

// Método coordinatedMovement que define el comportamiento del robot en el estado COORDINATED_MOVEMENT
void PairingRobot::coordinatedMovement(std::vector<mrs::RobotPtr> &swarm) {
    std::cout << "coordinatedMovement" << std::endl;
    timer++;
    // Asignar la velocidad ajustada con la alineación de ángulos
    m_vel= mrs::Velocity2d(-0.5*sin(coordinatedAngle), 0.5 *cos(coordinatedAngle));
    if (timer > 100 && state==COORDINATED_MOVEMENT ) { // después de 10 segundos
        state = REPULSION;
        timer = 0;
    }
}

// Método repulsion que define el comportamiento del robot en el estado REPULSION
void PairingRobot::repulsion(std::vector<mrs::RobotPtr> &swarm) {
    std::cout << "repulsion" << std::endl;
    timer++; // Incrementar el contador de tiempo
    float distance = mrs::distance(partnerPosition, m_pos); // Calcular la distancia al compañero
    float angle = mrs::angle(m_pos, partnerPosition); // Calcular el ángulo hacia el compañero
    // Aplicar una fuerza de repulsión en dirección opuesta
    float Kp = 1.0; // Ganancia proporcional para la fuerza de repulsión
    m_vel += mrs::Velocity2d(Kp * cos(angle+wanderAngle), Kp * sin(angle+wanderAngle));

    if (timer > 50 && distance > 2.0f) { // Si el otro robot ha salido del entorno de percepción y el tiempo es mayor a 50
        state = WANDERING; // Cambiar el estado a WANDERING
        timer = 0; // Reiniciar el contador de tiempo
    }
}

