# Projet_Batiment_ADS1220_ADS1262

-Contexte
Ce projet est mené dans le cadre des recherches.
Il s’inscrit dans le projet Campusco et vise à développer un prototype IoT permettant de monitorer en temps réel les performances thermiques des bâtiments.

-Objectifs
Mesurer de très petites variations de tension (µV voire nV) grâce à des ADC haute précision (ADS1220, ADS1262).

Tester différentes configurations d’ADC afin d’atteindre les objectifs de précision définis.

Connecter le système de mesure à ThingsBoard, déjà déployé dans le cadre du projet.

Consigner les valeurs dans une base de données et afficher des métriques sur un Dashboard interactif.

🛠️ Matériel
-ADC : ADS1220, ADS1262
-Microcontrôleurs : Raspberry Pi

-Capteurs thermiques : prévus dans la configuration finale, mais remplacés pour le moment par un potentiomètre afin de simuler les variations de tension

💻 Logiciel
Langages : C/C++, 

Bibliothèques : paho.mqtt.c, WiringPi, jsoncpp

Plateforme IoT : ThingsBoard (RPC, télémétrie, Dashboard)

🔗 Intégration réseau :
Connexion via MQTT à ThingsBoard

Gestion des RPC pour reconfigurer à distance les paramètres des ADC (PGA, Data Rate, etc.)

Visualisation des métriques sur Dashboard (température, Tension)

📊 Résultats :
Prototype fonctionnel capable de mesurer et transmettre des variations de tension.

Dashboard interactif pour le suivi en temps réel des performances thermiques des bâtiments.
