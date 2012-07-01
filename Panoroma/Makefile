SUBDIR= \
AlignPair \
CameraPose \
MatchPair \
WarpCylinderical

all:
	for i in $(SUBDIR) ; do \
	(cd $$i ; make) \
	done

clean:
	for i in $(SUBDIR) ; do \
	(cd $$i ; make clean) \
	done
