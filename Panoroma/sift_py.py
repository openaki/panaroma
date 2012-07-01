import sys
import os

j = 0;
for i in sys.argv[1:]:
    j +=1
    print i
    os.system("../sift <%s >%04d.key"%(i, j) )

