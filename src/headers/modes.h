#ifndef MODES_H
#define MODES_H


/**
 * @brief Executes the program in batch mode, reading input from files and computing the optimal route.
 *
 * This function reads the graph data from location and distance files, processes the input file to determine
 * the source, destination, and constraints, and then computes the optimal route based on the specified mode
 * (driving or eco mode).
 *
 * Input format: ./DA_PROJ1 <locations file> <distances file> <input file> <something>
 * Note: the last argument can be anything, only the number of arguments is relevant for this function.
 *
 * @param argv Command-line arguments containing the paths to the locations file, distances file, and input file.
 * @param argc Number of command line arguments.
 */
void batchMode(char *argv[], int argc);

/**
 * @brief Executes the program in manual mode, allowing the user to input data interactively.
 *
 * This function prompts the user to provide the locations and distances files, then allows them to choose
 * between driving mode and eco mode. Based on the user's inputs, it computes and displays the optimal route.
 */
void manualMode();

#endif //MODES_H
