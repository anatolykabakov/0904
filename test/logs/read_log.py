
import matplotlib.pyplot as plt
f = open("log2.txt", "r")

lines = []
x = []
y = []

for line in f:
    line = line.replace('\n', '')
    line = line.split(' ')
    print(str(line[1])+' '+str(line[2]))
    x.append(line[1])
    y.append(line[2])
    plt.plot(x, y,".b")
    plt.pause(0.001)
