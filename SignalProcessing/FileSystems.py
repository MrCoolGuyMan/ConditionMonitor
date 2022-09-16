from MainHeader import *
def findWhichSlashCharacter(InputFileName):

    #sometimes mr computer uses a \ and sometimes a /
    #it is absolutely critical that we do not use the wrong one

    slash_character = '\\'

    if InputFileName.find(slash_character) == -1:
        slash_character = '/'

    return slash_character

#find the final slash in a directory C:\\Desktop\FileName.csv
def findFinalSlashPosition(InputFileName):

    offset = 0
    slash_character = findWhichSlashCharacter(InputFileName)

    while InputFileName.find(slash_character,offset) != -1:
        offset = InputFileName.find(slash_character,offset)+1

    return  offset

def copyCSVfileToMem(filename):
    file = codecs.open(filename)#, encoding='UTF-8')
    RAMfile = np.loadtxt(file,dtype=float,delimiter=',', skiprows=0)
    return RAMfile