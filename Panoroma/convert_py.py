import sys
import os

j = 0;
for i in sys.argv[1:]:
    print i
    j +=1
    os.system("convert %s image%04d.pgm"%(i, j) )

