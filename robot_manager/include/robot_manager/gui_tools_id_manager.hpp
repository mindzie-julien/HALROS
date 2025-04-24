#ifndef GUI_TOOLS_ID_MANAGER_H
#define GUI_TOOLS_ID_MANAGER_H

#define MSG "MSG" // notification de demande/envoi de message
#define RAS "RAS" // notification de aucun processus en cours de traitement
#define OK "OK" // notification de validation d'état
#define STOP "READY" // notification de changement d'état possible
#define CANCEL "CANCEL" // notification de désactivation du changement d'état
#define START "START" // notification de lancement du nouvel état
#define STOP "STOP" // notification de désactivation d'état
#define END "END" // notification de fin d"état

/* 
- true si gui_manager utilise la communication UART GUI-ORDINATEUR, 
- false si gui_manager utilise les fichiers robot_json_path.json et robot_json_location.json, 
 pour commander le robot_manager
*/
#define GUI_ACTIVATED   false

#endif // GUI_TOOLS_ID_MANAGER_H