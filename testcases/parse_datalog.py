import sys

in_file = sys.argv[1]
out_file = sys.argv[2]
pp_file = sys.argv[3]

diff_list = list()

pseudo_print = list()

#define STEPS_PER_M_X            40000
#define STEPS_PER_M_Y            40000
#define STEPS_PER_M_Z            320000

def parse_stepper_position(line):
    s_line = line.split()
    
    X = float(s_line[1]) / 40.    # X-axis
    Y = float(s_line[2]) / 40.    # Y-axis
    Z = float(s_line[3]) / 320.   # Z-axis

    return X, Y, Z
    

def parse_m114_position(line):
    s_line = line.split(',')
    
    X = float(s_line[0][4:])
    Y = float(s_line[1][2:])
    Z = float(s_line[2][2:])
    
    return X, Y, Z

with open(in_file, 'r') as file:
    old_line = None
    found_m114 = False
    for _ in range(50):
        next(file)
    for i, line in enumerate(file):
        if found_m114:  # we found a M114 before, so next line is the result
            found_m114 = False
            x1, y1, z1 = parse_m114_position(line)
            x = x2 - x1
            diff_list.append('{}\t\t\t{}\t\t\t{}\t\t\t{}\n'.format(i, x1, x2, x))
            pseudo_print.append('{}\t\t\t{}\t\t\t{}\n'.format(x2, y2, z2))
        if line.find('M114') >= 0: # we found a M114, so use the line before as the result
            if old_line is not None:
                found_m114 = True
                x2, y2, z2 = parse_stepper_position(old_line)
        if len(line.split()) == 21: # sometimes we have empty lines. with this we are sure to have the right line.
            old_line = line

with open(out_file, 'w') as file:
    file.writelines(diff_list)

with open(pp_file, 'w') as file:
    file.writelines(pseudo_print)
