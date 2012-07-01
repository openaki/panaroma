import sys
import os

for i in range(int(sys.argv[1]), int(sys.argv[2])):
    print "Match pair", i, " ", i+1;
    os.system("../MatchPair/MatchPair -i %d -j %d"%(i, i+1) )

print "Match pair", i, " ", i+1;
os.system("../MatchPair/MatchPair -i %d -j %d"%(i+1, 1) )
