#include "pairing.hh" // Incluir la cabecera pairing.hh que contiene las declaraciones relacionadas con la clase PairingRobot
#include "environment.hh" // Incluir la cabecera environment.hh para las declaraciones del entorno
#include <cmath> // Incluir la librería cmath para las funciones matemáticas
#include <cstdlib> // Incluir la librería cstdlib para las funciones relacionadas con la generación de números aleatorios

// Constructor de la clase PairingRobot, inicializa los parámetros y configura el estado inicial como WANDERING
PairingRobot::PairingRobot(unsigned int id, const mrs::Position2d & p, 
                           const mrs::RobotSettings & settings,
                           const mrs::Velocity2d & vel)
    : mrs::Robot(id, p, settings), state(WANDERING), timer(0),coordinatedAngle(0.0f),desiredDistance(0.75f){}

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

// Método wander que define el comportamiento del robot en el estado WANDERING
void PairingRobot::wander(std::vector<mrs::RobotPtr> & swarm) {
    timer++; // Incrementar el contador de tiempo
    if ((timer % 100 == 0)) { // Cada 50 unidades de tiempo
        float angle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI; // Generar un ángulo aleatorio
        m_vel = mrs::Velocity2d(0.5f * cos(angle), 0.5f * sin(angle)); // Asignar una velocidad constante en una dirección aleatoria
    }

    // Verificar la cercanía de otros robots en el enjambre
    for (auto& neighbor : swarm) {
        if (!sameColor(neighbor) && mrs::distance(m_pos, neighbor->position()) < 1.0f) {
            // Si se encuentra un robot de diferente color a una distancia menor a 5.0
            partnerPosition = neighbor->position(); // Guardar la posición del compañero
            partnerColor[0] = neighbor->settings().color[0]; // Guardar el color del compañero
            partnerColor[1] = neighbor->settings().color[1];
            partnerColor[2] = neighbor->settings().color[2];
            state = COORDINATED_MOVEMENT; // Cambiar el estado a COORDINATED_MOVEMENT
            coordinatedAngle = mrs::angle(m_pos, partnerPosition) - M_PI_2; // M_PI_2 es 90 grados o π/2 radianes
            timer = 0; // Reiniciar el contador de tiempo
            break;
        }
    }
}

// Método coordinatedMovement que define el comportamiento del robot en el estado COORDINATED_MOVEMENT
void PairingRobot::coordinatedMovement(std::vector<mrs::RobotPtr> & swarm) {
    timer++;

    float distance = mrs::distance(m_pos, partnerPosition);
    float Kp = 10; // Ganancia del controlador proporciona
    float adjustment = Kp * (distance - desiredDistance);
    // Mantener el ángulo calculado al inicio del estado
    m_vel = mrs::Velocity2d(adjustment *cos(coordinatedAngle), adjustment * sin(coordinatedAngle)); // Velocidad constante

    if (timer > 500) { // después de 5-10 segundos
        std::cout << "REPULSION" << std::endl;
        state = REPULSION;
        timer = 0;
    }
}

// Método repulsion que define el comportamiento del robot en el estado REPULSION
void PairingRobot::repulsion(std::vector<mrs::RobotPtr> & swarm) {
    timer++; // Incrementar el contador de tiempo
    float distance = mrs::distance(m_pos, partnerPosition); // Calcular la distancia al compañero
    float desiredDistance = 2.0f; // Establecer una distancia de repulsión deseada
    float Kp = 10; // Incrementar la ganancia para una mayor fuerza de repulsión

    float adjustment = Kp * (desiredDistance - distance); // Ajuste de la velocidad basado en la distancia
    float angle = mrs::angle(m_pos, partnerPosition); // Calcular el ángulo hacia el compañero
    m_vel = mrs::Velocity2d(adjustment * cos(angle), adjustment * sin(-angle)); // Asignar la nueva velocidad

    if (distance >= desiredDistance) { // si el otro robot ha salido del entorno de percepción
        state = WANDERING; // Cambiar el estado a WANDERING
        timer = 0; // Reiniciar el contador de tiempo
    }
}
