import sys
import os

for i in sys.argv[1:]:
    j = int(i[4:8]) - 409
    k = "image%04d.jpg"%(j)
    print k;
    os.system("mv %s %s"%(i, k) );
