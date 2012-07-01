import sys
import os

for i in range(int(sys.argv[1]), int(sys.argv[2])):
    print "Align pair", i, " ", i+1;
    os.system("../AlignPair/AlignPair -i %d -j %d -tol 2.0 -iter 5000"%(i, i+1) )

print "Align pair", sys.argv[2], " ", 1;
os.system("../AlignPair/AlignPair -i %d -j %d -tol 2.0 -iter 5000"%(int(sys.argv[2]), 1) )
