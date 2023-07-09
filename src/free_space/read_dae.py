import numpy as np
import xml.etree.ElementTree as ET


def read_dae(path: str):
    # Parse the file
    tree = ET.parse(path)

    # Get the root element
    root = tree.getroot()

    # Check that the root element is a DAE
    if not root.tag.endswith("COLLADA"):
        raise RuntimeError(f"File {path} is not a COLLADA file.")

    namespace = {"ns": root.tag[1:-len("COLLADA") - 1]}
    sources = root.findall("ns:library_geometries/ns:geometry/ns:mesh/ns:source", namespace)

    # Get the xml positions elements
    positions = [
        position
        for source in sources if source.attrib["id"].endswith("positions")
        for position in source.findall("ns:float_array", namespace)
    ]

    # Read actual points
    points = np.array([
        float(point)
        for position in positions
        for point in position.text.strip().split()  # type: ignore
    ]).reshape(-1, 3)

    return points


if __name__ == "__main__":
    # Get the path to the file as the first argument
    import sys
    path = sys.argv[1]
    print(f"Received file: {path}")

    # Check that file exists
    import os
    if not os.path.isfile(path):
        raise RuntimeError(f"File {path} does not exist.")

    points = read_dae(path)

    print(points.shape, points)
