//
// Rejoue un fichier sauvegardé.
//
#include <librealsense2/rs.hpp>
#include <mutex>
#include <cstring>
#include "../run.h"

int main() {
    printf("Playback file:\n");
    char filename[1024];
    fgets(filename, 1024, stdin);

    strtok(filename, "\n"); // retrait du retour à la ligne à la fin du nom de fichier

    printf("Reading from file %s\n", filename);

    run(filename);
    return 0;

}