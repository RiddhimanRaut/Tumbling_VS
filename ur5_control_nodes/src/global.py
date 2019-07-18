x=0

def a():
    global x
    print x
    x=5
    
    print x

if __name__=='__main__':
    a()
    print x
