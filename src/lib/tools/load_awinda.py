import xml.etree.ElementTree as ET
import time


def test():
    test_str = "-0.893374 0.394140 0.893314 -0.898155 0.397323 0.989452 -0.893806 0.402549 1.094983 -0.900560 0.407550 1.191172 -0.908170 0.413084 1.287159 -0.905920 0.422061 1.421953 -0.882497 0.429315 1.508349 -0.905613 0.388779 1.364675 -0.901379 0.249933 1.362658 -0.869069 0.183459 1.074173 -0.853428 0.129253 0.837410 -0.908318 0.447643 1.360812 -0.908804 0.584541 1.337167 -0.917267 0.641265 1.044948 -0.887161 0.649352 0.803566 -0.891833 0.315254 0.896160 -0.849172 0.271858 0.491850 -0.954266 0.234724 0.109000 -0.797381 0.219523 0.017972 -0.894926 0.473033 0.890773 -0.904683 0.429400 0.484367 -0.944836 0.384044 0.090251 -0.776710 0.383148 0.015519"
    print(len(test_str.split()))


def read_awinda_mvnx(file_name: str):
    start = time.time()
    root = ET.parse(file_name).getroot()
    end = time.time()
    print("read in ", end - start)
    return root


def get_points_and_connections(root):
    points = []
    segments = root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}segments")
    for segment in segments:
        points.append(segment.attrib["label"])
    joints = root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}joints")
    connections = []
    for joint in joints:
        connection = []
        for connector in joint:
            connection.append(connector.text.split("/")[0])
        connections.append(connection)
    return (points, connections)


def main():
    root = read_awinda_mvnx("Test-007#Hannah2.mvnx")
    for child in root:
        print(child.tag)
    points, connections = get_points_and_connections(root)
    print(connections)
    print(points)
    point_posittions = dict(map(lambda x: (x, [0., 0., 0.]), points))
    print(point_posittions)
    frames = root.find("{http://www.xsens.com/mvn/mvnx}subject").find("{http://www.xsens.com/mvn/mvnx}frames")
    count = 0
    for frame in frames:
        print("#############################")
        if frame.attrib["type"] != "normal":
            continue
        positions = frame.find("{http://www.xsens.com/mvn/mvnx}position")
        positions = positions.text.split()
        if len(positions) % 3 != 0:
            print("[ERROR]: positions are not divisible by 3")
            break
        position_len = len(positions) // 3
        for i in range(position_len):
            point_posittions[positions[i]] = positions[i * 3: (i+1) * 3]
        print(point_posittions)
        count += 1
        if count > 5:
            break


if __name__ == "__main__":
    main()