import os
import tempfile
import shutil


# We need to use CMake add_custom_command() to download and `make` the LKH solver
LKH_CMD = "/opt/LKH-2.0.10/LKH"  # Path to the LKH solver's executable
LKH_FILES = "/tmp/LKH_files"   # Path to the temporary files directory


def lkh(distances, silent=True):
    # Generate a random filename in the /tmp/LKH_files folder
    if os.path.exists(LKH_FILES):
        shutil.rmtree(LKH_FILES)

    os.makedirs(LKH_FILES)

    with tempfile.NamedTemporaryFile(dir=LKH_FILES) as f:
        filename = f.name

    # Write the problem file
    with open(f"{filename}.tsp", "w") as f:
        # Write the header
        f.write(f"NAME : {filename}\n" +
                # f.write(f"COMMENT : {user_comment}\n")
                f"COMMENT : top_kek\n"
                "TYPE : ATSP\n" +
                f"DIMENSION : {len(distances)}\n" +
                "EDGE_WEIGHT_TYPE : EXPLICIT\n" +
                "EDGE_WEIGHT_FORMAT: FULL_MATRIX\n" +
                "EDGE_WEIGHT_SECTION\n")

        # Write the distance matrix
        for row in distances:
            cost_matrix_strline = " ".join(str(int(cost*10)) for cost in row)
            f.write(f"{cost_matrix_strline}\n")

        # Write the EOF
        f.write("EOF\n")

    # Write the parameter file for the run
    with open(f"{filename}.par", "w") as f:
        f.write(f"PROBLEM_FILE = {filename}.tsp\n" +
                "MOVE_TYPE = 5\n" +
                "PATCHING_C = 3\n" +
                "PATCHING_A = 2\n" +
                "RUNS = 3\n" +
                f"TOUR_FILE = {filename}.txt\n")

    # Run the LKH executable
    os.system(f"{LKH_CMD} {filename}.par" + (" > /dev/null" if silent else ""))

    # Read the result
    with open(f"{filename}.txt", "r") as f:
        lines = f.readlines()

    sequence = []
    # start reading from the 7-th line and terminate when -1 occurs
    i = 6
    while True:
        index = int(lines[i])
        if index == -1:
            break

        sequence.append(index - 1)
        i += 1

    # Remove our dir
    # shutil.rmtree(LKH_FILES)

    return sequence
