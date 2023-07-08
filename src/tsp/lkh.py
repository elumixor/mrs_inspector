import os
import tempfile
import shutil
from rospkg import RosPack

import rospy


__lkh_version = "2.0.10"
__lkh_installed = False
__lkh_path = None
__lkh_dir = None
__lkh_files = "/tmp/LKH_files"   # Path to the temporary files directory


def lkh(distances, silent=True):
    # Ensure LKH solver is installed
    __ensure_lkh()

    # Generate a random filename in the /tmp/LKH_files folder
    if os.path.exists(__lkh_files):
        shutil.rmtree(__lkh_files)

    os.makedirs(__lkh_files)

    with tempfile.NamedTemporaryFile(dir=__lkh_files) as f:
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
    os.system(f"{__lkh_path} {filename}.par" + (" > /dev/null" if silent else ""))

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


# This is not how it should be, but I'm not sure how to install it with catkin/cmake
def __ensure_lkh():
    global __lkh_installed, __lkh_path, __lkh_dir
    if __lkh_path is None:
        __lkh_dir = os.path.join(RosPack().get_path("mrs_inspector"), "resources/lkh")
        __lkh_path = os.path.join(__lkh_dir, f"LKH-{__lkh_version}/LKH")
        __lkh_installed = os.path.exists(__lkh_path)

    # Install LKH if it is not installed
    if not __lkh_installed:
        rospy.logwarn("LKH is not installed. Installing it now.")

        os.makedirs(__lkh_dir, exist_ok=True)  # type: ignore

        # Download the tarball
        rospy.loginfo("Downloading LKH")
        os.system(f"wget -O {__lkh_dir}/LKH-2.0.10.tgz http://akira.ruc.dk/~keld/research/LKH/LKH-{__lkh_version}.tgz")

        # Extract the tarball
        rospy.loginfo("Extracting LKH")
        os.system(f"tar -xzf {__lkh_dir}/LKH-2.0.10.tgz -C {__lkh_dir}")

        # Compile the LKH
        rospy.loginfo("Compiling LKH")
        os.system(f"make -C {__lkh_dir}/LKH-2.0.10")

        # Remove the tarball
        rospy.loginfo("Removing LKH tarball")
        os.system(f"rm {__lkh_dir}/LKH-2.0.10.tgz")

        # Check if all went well
        __lkh_installed = os.path.exists(__lkh_path)

        if not __lkh_installed:
            raise RuntimeError("Could not install LKH")
