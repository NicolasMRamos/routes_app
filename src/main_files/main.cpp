#include "../headers/modes.h"
#include <iostream>

// Main function -------------------------------------------------------------------------------------------------------

/**
 * @brief The main function of the program.
 *
 * This function determines whether to run the program in batch mode or manual mode based on the number of command-line arguments.
 * - If command-line arguments are provided, it runs in batch mode using the provided arguments.
 * - If no arguments are provided, it runs in manual mode, prompting the user for input interactively.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of command-line arguments.
 * @return int Returns 0 upon successful execution.
 */
int main(const int argc, char *argv[]) {
    if (argc >= 4)
        batchMode(argv, argc);
    else if (argc == 1)
        manualMode();
    else
      std::cerr << "Not enough arguments given.\n";
    return 0;
}

