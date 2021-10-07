#(find the location of your python installation, (where python.exe is))
#make a (windows (explorer)) shortcut to the python program,
#in the properties of the new shortcut, add the path to your python installation before the (in quates) path of the program, so:
#"C:\Users\user\Desktop\how to drag and drop files.py"
#becomes
#C:\Program Files\Python\python.exe "C:\Users\user\Desktop\how to drag and drop files.py"
#then you can drag and drop files onto the shortcut
#the dopped files will appear in the (list of) sys.argv, for which sys needs to be imported
#this file can be used as an example, it just prints the argv list and waits to be closed


import sys

if(len(sys.argv) > 0):
    print("first arg (program file location):", sys.argv[0])
if(len(sys.argv) > 1):
    print("second arg (drag-drop file location):", sys.argv[1])
print()
print("complete sys.argv (list):")
print(sys.argv)

#now just do nothing forever
var = 0
while(True):
    var += 1