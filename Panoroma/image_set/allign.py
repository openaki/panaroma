import sys
import os

for i in range(int(sys.argv[1]), int(sys.argv[2])):
    print "Match pair", i, " ", i+1;
    os.system("../AlignPair/AlignPair -i %d -j %d -tol 1.5 -iter 5000"%(i, i+1) )

print "Match pair", sys.argv[2], " ", 1;
os.system("../AlignPair/AlignPair -i %d -j %d -tol 1.5 -iter 5000"%(int(sys.argv[2]), 1) )
