import sys
import linecache

in_file = sys.argv[1]
out_file = sys.argv[2]
pp_file = sys.argv[3]

diff_list = list()

pseudo_print = list()

# define STEPS_PER_M_X            40000
# define STEPS_PER_M_Y            40000
# define STEPS_PER_M_Z            320000


def parse_stepper_position(line):
    s_line = line.split()

    X = float(s_line[1]) / 40.0  # X-axis
    Y = float(s_line[2]) / 40.0  # Y-axis
    Z = float(s_line[3]) / 320.0  # Z-axis

    return X, Y, Z


def parse_m114_position(line):
    s_line = line.split(",")

    X = float(s_line[0][4:])
    Y = float(s_line[1][2:])
    Z = float(s_line[2][2:])

    return X, Y, Z


with open(in_file, "r") as file:
    start_with_line = 50

    found_m114 = False
    for _ in range(start_with_line - 1):
        next(file)
    for i, line in enumerate(file, start_with_line):
        if found_m114:  # we found a M114 before, so next line is the result
            found_m114 = False
            x1, y1, z1 = parse_m114_position(line)
            x = x2 - x1
            diff_list.append("{}\t\t\t{}\t\t\t{}\t\t\t{}\n".format(i, x1, x2, x))
            pseudo_print.append("{}\t\t\t{}\t\t\t{}\n".format(x2, y2, z2))

        if line[0] == "#":
            if line[2:6] == "M114":
                found_m114 = True
                # find the line with stepping positions before the M114
                # print(linecache.getline(in_file, i))
                for x in range(i - 1, i - 20, -1):
                    pre_m114_line = linecache.getline(in_file, x)
                    if len(pre_m114_line.split()) == 21:
                        break
                x2, y2, z2 = parse_stepper_position(pre_m114_line)

with open(out_file, "w") as file:
    file.writelines(diff_list)

with open(pp_file, "w") as file:
    file.writelines(pseudo_print)
