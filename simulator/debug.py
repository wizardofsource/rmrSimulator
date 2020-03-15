def printd(s, *a, **kw):
    if debug:
        print(s, *a, **kw)

def pause():
    p = input("Press the <Enter> key to continue...")
