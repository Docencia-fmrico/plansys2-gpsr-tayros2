[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-8d59dc4de5201274e310e4c54b9627a8934c3b88527886e3b421487c677d23eb.svg)](https://classroom.github.com/a/j9y_86cr)
# plansys2_gpsr

Ejercicio 4 de Planificación y Sistemas Cognitivos 2023

En grupos de 4, haced una aplicación en ROS 2 usando PlanSys2 que use el dominio de la [Práctica 3](https://github.com/Docencia-fmrico/planning-exercise/blob/main/README.md). El robot debe poder realizar en un simulador, navegando con Nav2, goals similares a los siguientes:

(ordena_casa robot1)
(abrir_puerta puerta_principal)
(dar robot1 vaso_leche abuelita)
(dar robot1 medicina abuelita)

Puntuación (sobre 10):   
* +5 correcto funcionamiento en el robot simulado.
* +2 Readme.md bien documentado con videos.
* +2 CI con Test de los nodos BT
* -3 Warnings o que no pase los tests.

## Lanzamiento del comportamiento
- ros2 launch ir_robots simulation.launch
- ros2 launch br2_navigation tiago_navigation.launch.py
- ros2 launch plansys2_gpsr_tayros2 plansys2_gpsr.launch.py
- ros2 run plansys2_gpsr_tayros2 gpsr_controller_node
