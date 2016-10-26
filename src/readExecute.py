import os
import time
#import datetime
#import random
import sys
import glob
import math

timeStart   = time.time()
targetDir   = sys.argv[1]
fileList    = glob.glob( targetDir + "*.dat" ) 
numList     = len( fileList)
fileCommand = '../data/command.dat'
fileResults = '../data/results.dat'
fileBackup  = '../data/command.bak'
strSimulate = './jumphit'
strBackup1  = "cp " + fileCommand + ' ' + fileBackup
strBackup2  = "cp " + fileBackup + ' ' + fileCommand
strNumFile  = "There are " + str( numList ) + " command files as " + targetDir + "."

print strNumFile

#print strBackup1
os.system( strBackup1 )

for n in range( 0, numList ):
    str_n      = str( n)
    strCopy_c  = "cp " + fileList[n] + ' ' + fileCommand     
    strCopy_r  = "cp " + fileResults + ' ' + targetDir + str_n.zfill(4) + '_results.txt'

    #print strCopy_c
    #print strSimulate
    #print strCopy_r

    os.system( strCopy_c )
    os.system( strSimulate )
    os.system( strCopy_r )

    timeNow    = time.time() - timeStart
    strTimeNow = str( int( timeNow ))
    print "Simulation time: " + strTimeNow + " s. " + str( 100*n/numList ) + "%."

#print strBackup2
os.system( strBackup2 )

#print list[n]
#print n

#d         = datetime.datetime.today()
#DirName1  = str("{0:04d}".format(d.year)) + str("{0:02d}".format(d.month))  + str("{0:02d}".format(d.day))
#DirName2  = str("{0:02d}".format(d.hour)) + str("{0:02d}".format(d.minute)) + str("{0:02d}".format(d.second))
#cmd_Mkdir = "mkdir ../data/" + DirName1 + '/' + DirName2

#print cmd_Mkdir
#os.system(cmd_Mkdir)

#SrcFile = '../data/' + DirName1 + '/' + HourMinute + '/' + TargetFile
#print SrcFile

#for line in open(SrcFile, 'r'):
    #itemList = line[:-1].split('\t')
    #print itemList
    #cmd = "./hopper2 " + str(itemList[0]) + " " + str(itemList[1]) + " " + str(itemList[2]) + " " + DirName2    
    #print cmd
    #os.system(cmd)
    #time.sleep(waitTime)

timeEnd    = time.time() - timeStart
strTimeEnd = str( int( timeEnd ))
print "Simulation time: " + strTimeEnd + " s."



