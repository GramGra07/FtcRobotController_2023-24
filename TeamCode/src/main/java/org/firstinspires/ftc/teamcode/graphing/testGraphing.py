from matplotlib import pyplot as plt

fileWrite = "file.txt"
saveFile = "graph.png"


def formatFile(file):
    # make sure all lines are different from the previous line
    rFile = open(file, "r")
    lines = rFile.readlines()
    rFile.close()
    fileW = open(file, "w")
    prevLine = ""
    fileW.write("")
    for line in lines:
        if (line != prevLine):
            fileW.write(line)
            prevLine = line
    fileW.close()


def log(posex, posey):
    file = open(fileWrite, "a")
    file.write(str(posex))
    file.write(" ")
    file.write(str(posey))
    file.write("\n")
    file.close()


def graph(r_file, title):
    title = title.replace(" ", "_")
    plt.title(title)
    r_file = open(r_file, "r")
    lines = r_file.readlines()
    for line in lines:
        if (line == lines[0]):
            beginX, beginY = line.split()
        else:
            endX, endY = line.split()
            plt.plot([int(beginX), int(endX)], [int(beginY), int(endY)], color="black")
            beginX = endX
            beginY = endY
    for line in lines:
        if (line == lines[0]):
            lastX, lastY = line.split()
            plt.plot(int(lastX), int(lastY), 'o', color="green", markersize=6)
        elif (line == lines[-1]):
            x, y = line.split()
            plt.plot(int(x), int(y), 'o', color="red", markersize=6)
        else:
            x, y = line.split()
            plt.plot(int(x), int(y), 'o', color="black", markersize=1)
    r_file.close()
    if title == "":
        saveFile = "saved/graph.png"
    else:
        saveFile = "saved/"+title + ".png"
    plt.savefig(saveFile)
    plt.show()


def setup():
    formatFile(fileWrite)
    plt.axis("square")
    plt.axis([-72, 72, -72, 72])
    plt.xticks(range(-72, 73, 24))
    plt.yticks(range(-72, 73, 24))
    plt.axhline(y=0, color="k")
    plt.axvline(x=0, color="k")
    plt.axvline(x=72, color="k")
    plt.axhline(y=72, color="k")
    plt.axvline(x=-72, color="k")
    plt.axhline(y=-72, color="k")
    plt.axvline(x=24, color="k")
    plt.axhline(y=24, color="k")
    plt.axvline(x=-24, color="k")
    plt.axhline(y=-24, color="k")
    plt.axvline(x=48, color="k")
    plt.axhline(y=48, color="k")
    plt.axvline(x=-48, color="k")
    plt.axhline(y=-48, color="k")
    plt.axvline(x=72, color="k")
    plt.axhline(y=72, color="k")
    plt.axvline(x=-72, color="k")
    plt.axhline(y=-72, color="k")
    drawField()


def drawField():
    # x points, y points
    xPoints, yPoints = [-72,-24], [48,48]
    plt.plot(xPoints, yPoints, color="blue")
    xPoints, yPoints = [24, 72], [48, 48]
    plt.plot(xPoints, yPoints, color="red")
    xPoints, yPoints = [-24, 0], [48, 72]
    plt.plot(xPoints, yPoints, color="blue")
    xPoints, yPoints = [0, 24], [72, 48]
    plt.plot(xPoints, yPoints, color="red")
    xPoints, yPoints = [-72,-24], [-12, -12]
    plt.plot(xPoints, yPoints, color="blue")
    xPoints, yPoints = [24, 72], [-12, -12]
    plt.plot(xPoints, yPoints, color="red")
    xPoints, yPoints = [-24, 24], [-12, -12]
    plt.plot(xPoints, yPoints, color="yellow")
    xPoints, yPoints = [-72, -48], [-6, -6]
    plt.plot(xPoints, yPoints, color="yellow")
    xPoints, yPoints = [-48, -24], [-18, -18]
    plt.plot(xPoints, yPoints, color="yellow")
    xPoints, yPoints = [24, 48], [-18, -18]
    plt.plot(xPoints, yPoints, color="yellow")
    xPoints, yPoints = [48, 72], [-6, -6]
    plt.plot(xPoints, yPoints, color="yellow")
    xPoints, yPoints = [-72, -48], [-48, -72]
    plt.plot(xPoints, yPoints, color="red")
    xPoints, yPoints = [72, 48], [-48, -72]
    plt.plot(xPoints, yPoints, color="blue")
    xPoints, yPoints = [-24, 24], [-6, -6]
    plt.plot(xPoints, yPoints, color="yellow")
    xPoints, yPoints = [-24, -24], [0, -24]
    plt.plot(xPoints, yPoints,color="grey")
    xPoints, yPoints = [-48, -48], [0, -24]
    plt.plot(xPoints, yPoints,color="grey")
    xPoints, yPoints = [24, 24], [0, -24]
    plt.plot(xPoints, yPoints,color="grey")
    xPoints, yPoints = [48, 48], [0, -24]
    plt.plot(xPoints, yPoints,color="grey")
    xPoints, yPoints = [-72, -72], [0, -24]
    plt.plot(xPoints, yPoints,color="grey")
    xPoints, yPoints = [72, 72], [0, -24]
    plt.plot(xPoints, yPoints,color="grey")


setup()
graph(fileWrite,"dec7Match10")
# title is optional, if you want it to be saved,
# put a title in, otherwise leave blank ("")