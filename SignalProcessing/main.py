from MainHeader import *    #* means import everything
"""
FileNames contains a string of filenames separated by '\n' characters
"""
def main(FileNames:str,Filter:FilterClass=None,SelectedMotor:MotorClass=None):

    if (Filter == None):
        Filter = FilterClass()  # default values

    #fileNames could be one or more file names seperated by a '\n' character
    if FileNames.find('\n') == -1:
        FileNames += '\n'           #in case some joker manually types out a file name

    previousindex = 0

    #find each file name in turn
    while FileNames.find('\n',previousindex) != -1:
        index = FileNames.find('\n',previousindex)

        if FileNames[previousindex:index].find('csv') != -1:
            mainAnalysis(FileNames[previousindex:index], Filter,SelectedMotor)

        previousindex = index + 1

    return;

#this has to be at the end of the file
if __name__ == '__main__':
    GUI = MAIN_PAGE()
    GUI.toplevel.mainloop()


#Command to make into executable
#pyinstaller --onefile --name ConditionMonitoring main.py
