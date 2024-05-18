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
    timer++; // Incrementar el contador de tiempo
    if ((timer % 100 == 1)) { // Cada 50 unidades de tiempo
        wanderAngle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI; // Generar un ángulo aleatorio
        m_vel = mrs::Velocity2d(0.5f * cos(wanderAngle), 0.5f * sin(wanderAngle)); // Asignar una velocidad constante en una dirección aleatoria
    }

    // Verificar la cercanía de otros robots en el enjambre
    for (auto& neighbor : swarm) {

        if (!sameColor(neighbor) && mrs::distance(m_pos, neighbor->position()) < 2.0f) {
            // Si se encuentra un robot de diferente color a una distancia menor a 5.0
            partnerPosition = neighbor->position(); // Guardar la posición del compañero
            partnerColor[0] = neighbor->settings().color[0]; // Guardar el color del compañero
            state = COORDINATED_MOVEMENT; // Cambiar el estado a COORDINATED_MOVEMENT
            timer = 0; // Reiniciar el contador de tiempo
            coordinatedAngle =mrs::angle(m_pos, partnerPosition);; // Calcular el ángulo hacia el compañero
            std::cout << "Robot: " << i << " Coordinated Movement" << std::endl;
            // Calcular la distancia actual al compañero
            std::cout << "m_pos: " << m_pos << std::endl;
            std::cout << "partnerPosition: " << partnerPosition << std::endl;
            std::cout << "coordinated: " << coordinatedAngle << std::endl;
            std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
            i+=1;
            break;
        }
    }
}

// Método coordinatedMovement que define el comportamiento del robot en el estado COORDINATED_MOVEMENT
void PairingRobot::coordinatedMovement(std::vector<mrs::RobotPtr> & swarm) {
    timer++;
    
    // Asignar la velocidad ajustada con la alineación de ángulos
    if(coordinatedAngle > 0)
        m_vel= mrs::Velocity2d(-0.5*sin(coordinatedAngle), 0.5 *cos(coordinatedAngle));
    else if (coordinatedAngle < 0)
    {
        m_vel= mrs::Velocity2d(-0.5*sin(-coordinatedAngle), 0.5*cos(-coordinatedAngle));
    }
    



    if (timer > 100) { // después de 10 segundos
        state = REPULSION;
        timer = 0;
    }

}
// Método repulsion que define el comportamiento del robot en el estado REPULSION
void PairingRobot::repulsion(std::vector<mrs::RobotPtr> & swarm) {
    timer++; // Incrementar el contador de tiempo
    float distance = mrs::distance(m_pos, partnerPosition); // Calcular la distancia al compañero
    float desiredDistance = 3.0f; // Establecer una distancia de repulsión deseada
    float Kp = 1000; // Incrementar la ganancia para una mayor fuerza de repulsión

    float adjustment = Kp * (desiredDistance - distance); // Ajuste de la velocidad basado en la distancia
    float angle = mrs::angle(m_pos, partnerPosition); // Calcular el ángulo hacia el compañero
    m_vel += mrs::Velocity2d(adjustment * cos(-angle), adjustment * sin(-angle)); // Asignar la nueva velocidad

    if (distance >= desiredDistance && timer > 50 ) { // si el otro robot ha salido del entorno de percepción
        state = WANDERING; // Cambiar el estado a WANDERING
        timer = 0; // Reiniciar el contador de tiempo
    }
}
