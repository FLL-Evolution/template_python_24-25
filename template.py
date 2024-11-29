# Bibliothèques nécessaires pour que le code fonctionne
from hub import light_matrix, port, motion_sensor
import runloop, motor, color, color_sensor, math, time, motor_pair

# Modifier les ports selon vos branchements ici
port_moteur_roue_droite = port.D
port_moteur_roue_gauche = port.C
port_moteur_module_gauche = port.A
port_moteur_module_droit = port.B
port_capteur_couleur_droit = port.F
port_capteur_couleur_gauche = port.E

# Informations sur le robot
circonference_roue = 17.5

roues = 0
motor_pair.pair(roues, port_moteur_roue_gauche, port_moteur_roue_droite)

# Fonction permettant au robot d'avancer d'une certaine distance en ligne droite à l'aide du gyroscope et d'un contrôleur PID
def bouger_CM(distance):


    motor.reset_relative_position(port_moteur_roue_droite, 0)
    motor.reset_relative_position(port_moteur_roue_gauche, 0)

    time.sleep_ms(250)
    motion_sensor.reset_yaw(0)
    time.sleep_ms(250)
    gyro = motion_sensor.tilt_angles()

    PV = 0

    # Définition du setpoint en degrées
    SP = distance / 17.5 * 360
    t = 0
    # Initialisation des paramètres du PID
    eD_prev = 0
    t_prev = -100
    I = 0
    Kp = 3.0
    Ki = 0.1
    Kd = 0.5
    beta = 1
    gamma = 0
    I_max = 100# Limite pour l'intégrale

    while(abs(SP-PV) > 3):
        t += 1
        # Calcule de la distance en degrées parcouru par la roue depuis le début
        PV = (motor.relative_position(port_moteur_roue_droite) - motor.relative_position(port_moteur_roue_gauche)) / 2

        # Calcul du PID
        P = Kp*(beta*SP - PV)
        I = I + Ki*(SP - PV)*(t - t_prev)
        I = max(min(I, I_max), -I_max)# Limite l'intégrale
        eD = gamma*SP - PV
        D = Kd*(eD - eD_prev)/(t - t_prev)
        MV = P + I + D

        # Mise à jour des paramètres du PID
        eD_prev = eD
        t_prev = t

        #Ajoute la correction du gyro pour assurer une ligne droite
        motor_pair.move_tank(roues, int(MV + gyro[0]), int(MV - gyro[0]))


# Arrete les roues une fois rendu a destination
    motor_pair.stop(roues)

# Fonction permettant au robot de tourner selon un angle précis à l'aide du gyroscope et d'un contrôleur PID
def tourner_gyro(angle):
    time.sleep_ms(250)
    motion_sensor.reset_yaw(0)
    time.sleep_ms(250)
    gyro = motion_sensor.tilt_angles()
    PV = gyro[0]

    # Définition du setpoint
    SP = -angle * 10
    t = 0

    eD_prev = 0
    t_prev = -100
    I = 0
    Kp = 0.35
    Ki = 0.00005
    Kd = 0.15
    beta = 1
    gamma = 0
    I_max = 50 # Limite pour l'intégrale
    dead_zone = 25 # Zone morte pour éviter les petites oscillations

    while(abs(SP - PV) > dead_zone):
        t += 1

        gyro = motion_sensor.tilt_angles()
        PV = gyro[0]
        print(PV)

        # Calcul du PID
        P = Kp*(beta*SP - PV)
        I = I + Ki*(SP - PV)*(t - t_prev)
        I = max(min(I, I_max), -I_max)# Limite l'intégrale
        eD = gamma*SP - PV
        D = Kd*(eD - eD_prev)/(t - t_prev)
        MV = P + I + D

        # Mise à jour des paramètres du PID
        eD_prev = eD
        t_prev = t


        motor_pair.move_tank(roues, -int(MV), int(MV))

# Arrête les roues une fois rendu a destination
    motor_pair.stop(roues)

async def main():
    # Écrire votre code ici
    bouger_CM(-2)
    bouger_CM(30)
    tourner_gyro(-50)
    bouger_CM(20)
    bouger_CM(-20)
    tourner_gyro(50)
    motor.run_for_degrees(port_moteur_module_gauche, -90, 1110)
    time.sleep_ms(500)
    motor.run_for_degrees(port_moteur_module_gauche, 90, 1110)
    time.sleep_ms(500)
    motor.run_for_degrees(port_moteur_module_droit, -90, 1110)
    time.sleep_ms(500)
    motor.run_for_degrees(port_moteur_module_droit, 90, 1110)

runloop.run(main())
