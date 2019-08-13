import numpy as np

def get_numbers_from_string(string_var):
    # Split by space
    space_list = string_var.split(" ")
    
    # Saves numbers
    final_numbers = []

    # Try convert each word to a number
    for s in space_list:
        # Remove comma
        s = s.strip(",")
        s = s.strip("[")
        s = s.strip("]\n")
        try:
            number = float(s)
            final_numbers.append(number)
        except:
            pass

    return final_numbers

def lineseg_dist(p, a, b):
    # normalized tangent vector
    d = np.divide(b - a, np.linalg.norm(b - a))

    # signed parallel distance components
    s = np.dot(a - p, d)
    t = np.dot(p - b, d)

    # clamped parallel distance
    h = np.maximum.reduce([s, t, 0])

    # perpendicular distance component
    c = np.cross(p - a, d)

    return np.hypot(h, np.linalg.norm(c))


def get_numbers_after_string(file_name, the_string):

    nums = []

    # Get details from the simulation data
    file = open(file_name, "r")
    for line in file:
        # Find the total time
        if the_string in line:
            nums.append(get_numbers_from_string(line))
    file.close()

    return nums