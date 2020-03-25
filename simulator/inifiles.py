import sys

def readini(filename):
    res = {}
    section = ""

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip().replace(" ", "")
            if line.startswith("[") and line.endswith("]"):
                section = line[1:-1].lower()
                continue
            if "=" in line:
                x = line.split("=")
                if len(x) != 2:
                    print("Error at attribute %s multiple = in one line" % (x[0], ))
                    sys.exit(1)
                k,v = x
                if section not in res:
                    res[section] = {}
                res[section][k.lower()] = v
    return res

